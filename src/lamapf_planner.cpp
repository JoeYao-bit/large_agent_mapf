#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "common_interfaces.h"

#include "controller.h"
#include <gtest/gtest.h>
#include "common_interfaces.h"
#include "fake_agents.h"

//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

CenteralController* ctl = nullptr;


class InitExecutionSubscriber : public rclcpp::Node
{
public:
  InitExecutionSubscriber()
  : Node("path_execution_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "init_exe", 10, std::bind(&InitExecutionSubscriber::topic_callback, this, _1));
  }

private:

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s', reset controller progress", msg->data.c_str());
    sub_count_ ++;
    if(ctl != nullptr) {
        ctl->clearAllProgress();
    }
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];
        const auto& start_pt = instances.second[id].first.pt_; 
        auto color = COLOR_TABLE[id%30];

        geometry_msgs::msg::Pose initial_pose;

        Pointf<3> ptf = GridToPtf(start_pt);

        initial_pose.position.x = ptf[0];  // 设置模型初始位置
        initial_pose.position.y = ptf[1];
        initial_pose.position.z = agent_height + sub_count_*0.1;

        allAgentPoses[id][0] = initial_pose.position.x;
        allAgentPoses[id][1] = initial_pose.position.y;
        allAgentPoses[id][2] = ptf[2];
    }

  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// set all poses of agent in gazebo to allAgentPoses
void updateAllAgentPoseInGazebo(const SetPoseClientPtr& set_pose_clinet,
                                const rclcpp::Node::SharedPtr& node) {
    assert(allAgentPoses.size() == instances.second.size());
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];

        double target_x = allAgentPoses[id][0],
               target_y = allAgentPoses[id][1],
               target_z = agent_height,
               target_theta = allAgentPoses[id][2];
        if(agent->type_ == "Circle") {

            setModelPose(std::string("Circle_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet, node);

        } else if(agent->type_ == "Block_2D") {
           
            setModelPose(std::string("Block_2D_")+std::to_string(agent->id_), target_x, target_y, target_z, target_theta, set_pose_clinet, node);

        }
    }
}

void layeredLargeAgentMAPFTest(const std::string& file_path,
                               const rclcpp::Node::SharedPtr& node) {

    // 创建服务客户端
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
                node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // 等待服务可用
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "等待 /spawn_entity 服务...");
    }

    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_pose_clinet =
                node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state"); // /gazebo/set_entity_state

    // 等待服务可用
    while (!set_pose_clinet->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "等待 /gazebo/set_entity_state 服务...");
    }

    // subscribe global grid map 
    auto init_exe_sub = std::make_shared<InitExecutionSubscriber>();


    InstanceDeserializer<2> deserializer;
    if (deserializer.loadInstanceFromFile(file_path, dim)) {
        std::cout << "load from path " << file_path << " success" << std::endl;
    } else {
        std::cout << "load from path " << file_path << " failed" << std::endl;
        return;
    }
    std::cout << "map scale = " << dim[0] << "*" << dim[1] << std::endl;

    auto start_t = clock();

    LargeAgentMAPFInstanceDecompositionPtr<2> decomposer_ptr = nullptr;
    std::vector<std::vector<int> > grid_visit_count_table;

    instances = deserializer.getTestInstance({20}, 1).front();


    // add instance to gazebo
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;  // 设置模型初始位置
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.5;
    initial_pose.orientation.w = 1.0;


    global_offset_x = -0.5*dim[0]*reso, global_offset_y = -0.5*dim[1]*reso;
    // add grid map as a big block agent
    std::string file_path3 = createBlockAgent(std::string("ground"), 
                                            {-0.5*dim[0]*reso, -0.5*dim[1]*reso},
                                            { 0.5*dim[0]*reso,  0.5*dim[1]*reso}, .1, cv::Vec3b::all(200));

    initial_pose.position.x = 0;  // 设置模型初始位置
    initial_pose.position.y = 0;
    initial_pose.position.z = 0.05;

    spawnAgentGazebo(file_path3, std::string("ground"), initial_pose, client, node);

    allAgentPoses.resize(instances.second.size(), {0,0,0}); // x, y, yaw
    allAgentVels.resize(instances.second.size(), {0,0,0}); // x, y, yaw

    // add agent to gazebo
    for (int id=0; id<instances.first.size(); id++) {
        const auto& agent = instances.first[id];
        const auto& start_pt = instances.second[id].first.pt_; 
        auto color = COLOR_TABLE[id%30];
        double start_theta = orientToRadius(instances.second[id].first.orient_);

        Pointf<3> ptf = GridToPtf(start_pt);

        initial_pose.position.x = ptf[0];  // 设置模型初始位置
        initial_pose.position.y = ptf[1];
        initial_pose.position.z = .15;

        allAgentPoses[id][0] = initial_pose.position.x;
        allAgentPoses[id][1] = initial_pose.position.y;
        allAgentPoses[id][2] = start_theta;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, start_theta); // Create this quaternion from roll/pitch/yaw (in radians)

        initial_pose.orientation.x = orientation.x();
        initial_pose.orientation.y = orientation.y();
        initial_pose.orientation.z = orientation.z();
        initial_pose.orientation.w = orientation.w();

        if(agent->type_ == "Circle") {

            auto circle_agent_ptr = std::dynamic_pointer_cast<CircleAgent<2> >(agent);
            std::string file_path2 = createCircleAgent(std::string("Circle_")+std::to_string(agent->id_), 
                                                       circle_agent_ptr->radius_*reso, initial_pose.position.z, color);

            spawnAgentGazebo(file_path2, std::string("Circle_")+std::to_string(agent->id_), initial_pose, client, node);

        } else if(agent->type_ == "Block_2D") {

            auto block_agent_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent);
            std::string file_path3 = createBlockAgent(std::string("Block_2D_")+std::to_string(agent->id_), 
                                                      {block_agent_ptr->min_pt_[0]*reso, block_agent_ptr->min_pt_[1]*reso},
                                                      {block_agent_ptr->max_pt_[0]*reso, block_agent_ptr->max_pt_[1]*reso}, initial_pose.position.z, color);

            spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, client, node);

        }
    }

    auto layered_paths = layeredLargeAgentMAPF<2>(instances.second,
                                                  instances.first,
                                                  dim, is_occupied,
                                                  CBS::LargeAgentCBS_func<2>,
                                                  grid_visit_count_table,
                                                  60, decomposer_ptr,
                                                  false);

    auto end_t = clock();

    double time_cost = ((double)end_t-start_t)/CLOCKS_PER_SEC;

    std::cout << (layered_paths.size() == instances.first.size() ? "success" : "failed")
              << " layered large agent mapf in " << time_cost << "s " << std::endl;
    std::cout << std::endl;

    rclcpp::WallRate loop_rate(control_frequency);

    std::vector<LineFollowControllerPtr> line_ctls(instances.first.size(), std::make_shared<ConstantLineFollowController>(MotionConfig()));
    std::vector<RotateControllerPtr> rot_ctls(instances.first.size(), std::make_shared<ConstantRotateController>(MotionConfig()));

    std::cout << "decomposer_ptr->getAllPoses().size() = " << decomposer_ptr->getAllPoses().size() << std::endl;

    ctl = new CenteralController(layered_paths, instances.first, decomposer_ptr->getAllPoses(), line_ctls, rot_ctls);


    // InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
    //                        instances.front().second, layered_paths, grid_visit_count_table);

    // start a indenpendent thread to visualize all path
    // std::thread canvas_thread(InstanceVisualization, instances.first, decomposer_ptr->getAllPoses(),
    //                        instances.second, layered_paths, grid_visit_count_table);        

    // canvas_thread.detach();
    
    while (rclcpp::ok())
    {   
        // std::cout << "flag 1" << std::endl;
        RCLCPP_INFO(node->get_logger(), "control loop");

        Pointfs<3> new_vels = ctl->calculateCMD(allAgentPoses, allAgentVels, 1/control_frequency);
        // std::cout << "flag 2" << std::endl;

        for(int i=0; i<instances.first.size(); i++) {
            allAgentPoses[i] = updateAgentPose(allAgentPoses[i], new_vels[i], 1/control_frequency);
            // break;
        }
        // std::cout << "flag 3" << std::endl;

        updateAllAgentPoseInGazebo(set_pose_clinet, node);
        // std::cout << "flag 4" << std::endl;

        allAgentVels = new_vels;

        rclcpp::spin_some(init_exe_sub);
        loop_rate.sleep();
    }

    delete ctl;

    // TODO: set all agent to target



//    gettimeofday(&tv_pre, &tz);
//    CBS::LargeAgentCBS<2, CircleAgent<2> > solver(deserializer.getInstances(), deserializer.getAgents(),
//                                                  dim, is_occupied);
//    gettimeofday(&tv_after, &tz);
//    double time_cost1 = (tv_after.tv_sec - tv_pre.tv_sec) * 1e3 + (tv_after.tv_usec - tv_pre.tv_usec) / 1e3;
//    std::vector<LAMAPF_Path> raw_path;
//    if(solver.solve(60)) {
//        raw_path = solver.getSolution();
//    }
//    std::cout << (raw_path.size() == deserializer.getAgents().size() ? "success" : "failed")
//              << " raw large agent mapf in " << time_cost1 << "ms " << std::endl;

    // InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
    //                       instances.front().second, layered_paths, grid_visit_count_table);
}

std::vector<Pointf<3> > allAgentPoses;
std::vector<Pointf<3> > allAgentVels;

std::pair<AgentPtrs<2>, InstanceOrients<2> >  instances;


int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"), node);



    return 0;
}