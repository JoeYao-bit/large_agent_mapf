#ifndef LAMAPF_CONTROLLER
#define LAMAPF_CONTROLLER

#include "common_interfaces.h"
#include "action_dependency_graph.h"

// m/s, rad/s
struct MotionConfig {
    double max_v_x = .5, min_v_x = 0;
    double max_v_y = 0, min_v_y = 0;
    double max_v_w = .25*M_PI, min_v_w = -.25*M_PI;

    double max_a_x = .5, min_a_x = -0.5;
    double max_a_y = 0, min_a_y = 0;
    double max_a_w = .25*M_PI, min_a_w = -.25*M_PI;

    bool is_nonholonomic = true;
};

// there are two kinds of controller, follow a line and rotate
class LineFollowController {
public:
    // move from pt1 to pt2
    LineFollowController(const MotionConfig& cfg, const Pointf<2>& pt1, const Pointf<2>& pt2) {
        cfg_ = cfg;
        pt1_ = pt1;
        pt2_ = pt2;
    }

    // pose: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const = 0;

    MotionConfig cfg_;
    Pointf<2> pt1_, pt2_;

};

typedef std::shared_ptr<LineFollowController> LineFollowControllerPtr;


class ConstantLineFollowController : public LineFollowController {
public:

    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const override {
        Pointf<3> retv = {0, 0, 0};
        double dist_to_end = sqrt(pow(pose[0]-pt2_[0], 2) + pow(pose[1]-pt2_[1], 2));
        if(dist_to_end > cfg_.max_v_x*time_interval) {
            retv[0] = cfg_.max_v_x;
        } else {
            retv[0] = dist_to_end/time_interval;
        }
        return retv;
    }
};


class RotateController {
 public:
   
    // rotate from ang1 to ang2
    RotateController(const MotionConfig& cfg, bool posi_rot, float ang) {
        cfg_ = cfg;
        posi_rot_ = posi_rot;
        ang_ = ang;
        assert(ang >= 0 && ang <= 2*M_PI);
    }

    // poseposi_rot: x, y, theta / vel: x, y, theta
    virtual Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const = 0;

    MotionConfig cfg_;
    bool posi_rot_;
    float ang_;

};

typedef std::shared_ptr<RotateController> RotateControllerPtr;

class ConstantRotateController : public RotateController {
public:

    Pointf<3> calculateCMD(Pointf<3> pose, Pointf<3> vel, float time_interval) const override {
        Pointf<3> retv = {0, 0, 0};
        if(posi_rot_) {
            while(pose[2] > ang_) {
                pose[2] = pose[2] - 2*M_PI;
            }
            if(pose[2] + cfg_.max_v_w*time_interval < ang_) {
                retv[2] = cfg_.max_v_w;
            } else {
                retv[2] = (ang_ - pose[2])/time_interval;
            }
        } else {
            while(pose[2] < ang_) {
                pose[2] = pose[2] + 2*M_PI;
            }
            if(pose[2] + cfg_.min_v_w*time_interval > ang_) {
                retv[2] = cfg_.min_v_w;
            } else {
                retv[2] = (ang_ - pose[2])/time_interval;
            }
        }
        return retv;
    }

};



Pointf<3> updateAgentPose(Pointf<3> pose, Pointf<3> velcmd, float time_interval) {
    double new_x = pose[0] + time_interval*velcmd[0], 
           new_y = pose[1] + time_interval*velcmd[1],
           new_theta = fmod((pose[2] + time_interval*velcmd[2]) + 2*M_PI, 2*M_PI);
    return Pointf<3>{new_x, new_y, new_theta};
};

struct CenteralController {

    CenteralController(const std::vector<LAMAPF_Path>& paths,
                       const std::vector<AgentPtr<2>>& agents,
                       const std::vector<PosePtr<int, 2> >& all_poses,
                       const std::vector<LineFollowControllerPtr>& line_ctls,
                       const std::vector<RotateControllerPtr>& rot_ctls): 
                       ADG_(paths, agents, all_poses),
                       line_ctls_(line_ctls),
                       rot_ctls_(rot_ctls) {

        progress_of_agents_.resize(agents.size(), 0);

        assert(paths.size() == agents.size());
        assert(line_ctls.size() == agents.size());
        assert(rot_ctls.size() == agents.size());

    }

    // calculate vel cmd for each agent
    Pointfs<3> calculateCMD(Pointfs<3> poses, Pointfs<3> vels, float time_interval) {
        Pointfs<3> retv(poses.size(), {0, 0, 0});
        Pointf<3> cmd_vel;
        Pointf<3> start_ptf, target_ptf;
        for(int i=0; i<ADG_.agents_.size(); i++) {
            Pointf<3> cur_pose = poses[i];
            size_t target_pose_id;
            PosePtr<int, 2> target_pose;
            // update progress, considering agent may wait at current position multiple times
            while(true) {
                if(progress_of_agents_[i] < ADG_.paths_[i].size()) {
                    target_pose_id = ADG_.paths_[i][progress_of_agents_[i] + 1];
                    target_pose = ADG_.all_poses_[target_pose_id];
                    target_ptf = PoseIntToPtf(target_pose);
                    
                    float dist_to_target = (Pointf<2>{target_ptf[0], target_ptf[1]} - Pointf<2>{cur_pose[0], cur_pose[1]}).Norm();
                    float angle_to_target = fmod(orientToPi_2D(target_pose->orient_)-cur_pose[2], 2*M_PI);
                    // check whether reach current target, if reach, update progress to next pose
                    if(dist_to_target < 0.001 && angle_to_target < 0.001) {
                        // reach current target, move to next target
                        progress_of_agents_[i]++;
                    } else {
                        break;
                    }
                }
            }
            if(progress_of_agents_[i] < ADG_.paths_[i].size()) {
                // if not finish all pose, get cmd move to next pose, otherwise stop 
                size_t start_pose_id = ADG_.paths_[i][progress_of_agents_[i] + 1];;
                PosePtr<int, 2> start_pose = ADG_.all_poses_[start_pose_id];
                start_ptf = PoseIntToPtf(start_pose);
                // check whether need wait, via ADG
                retv[i] = Pointf<3>{0, 0, 0}; 

                // if move to next pose, current pose and next pose must be different 
                assert(start_pose_id != target_pose_id);

                if(start_pose->pt_ == target_pose->pt_) {
                    // rotate
                    auto& rot_ctr = rot_ctls_[i];
                    rot_ctr->ang_ = start_ptf[2]; // update target angle
                    cmd_vel = rot_ctr->calculateCMD(poses[i], vels[i], time_interval);

                } else {
                    // move forward
                    auto& line_ctr = line_ctls_[i];
                    line_ctr->pt1_ = Pointf<2>({start_ptf[0],  start_ptf[1]});
                    line_ctr->pt2_ = Pointf<2>({target_ptf[0], target_ptf[1]}); // update target line
                    cmd_vel = line_ctr->calculateCMD(poses[i], vels[i], time_interval);
                }
                retv[i] = cmd_vel; 
            }
            
        }
        return retv;
    }

    ActionDependencyGraph<2> ADG_;

    std::vector<LineFollowControllerPtr> line_ctls_;
    std::vector<RotateControllerPtr> rot_ctls_;

    std::vector<int> progress_of_agents_;

};


#endif





