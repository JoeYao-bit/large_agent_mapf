#include <cstdio>
// #include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>

#include "common_interfaces.h"


#include <gtest/gtest.h>
#include "common_interfaces.h"
#include "LA-MAPF/algorithm/LA-MAPF/laryered_large_agent_mapf.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/layered_large_agent_CBS.h"
#include "LA-MAPF/algorithm/LA-MAPF/CBS/large_agent_CBS.h"

#include "fake_agents.h"
//#include "../../algorithm/LA-MAPF/LaCAM/large_agent_lacam.h"

using namespace freeNav::LayeredMAPF::LA_MAPF;

void layeredLargeAgentMAPFTest(const std::string& file_path, 
                               const SpawnClientPtr& client, 
                               const SetPoseClientPtr& set_pose_clinet,
                               const rclcpp::Node::SharedPtr& node) {

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

    auto instances = deserializer.getTestInstance({20}, 1);


    // add instance to gazebo
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;  // 设置模型初始位置
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.5;
    initial_pose.orientation.w = 1.0;


    double reso = 0.05;
    double global_offset_x = -0.5*dim[0]*reso, global_offset_y = -0.5*dim[1]*reso;
    // add grid map as a big block agent
    std::string file_path3 = createBlockAgent(std::string("ground"), 
                                            {-0.5*dim[0]*reso, -0.5*dim[1]*reso},
                                            { 0.5*dim[0]*reso,  0.5*dim[1]*reso}, .1, cv::Vec3b::all(200));

    initial_pose.position.x = 0;  // 设置模型初始位置
    initial_pose.position.y = 0;
    initial_pose.position.z = 0.05;

    spawnAgentGazebo(file_path3, std::string("ground"), initial_pose, client, node);


    // add agent to gazebo
    for (int id=0; id<instances.front().first.size(); id++) {
        const auto& agent = instances.front().first[id];
        const auto& start_pt = instances.front().second[id].first.pt_; 
        auto color = COLOR_TABLE[id%30];
        double start_theta = orientToRadius(instances.front().second[id].first.orient_);

        initial_pose.position.x = global_offset_x + reso*start_pt[0];  // 设置模型初始位置
        initial_pose.position.y = global_offset_y + reso*start_pt[1];
        initial_pose.position.z = .15;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, start_theta); // Create this quaternion from roll/pitch/yaw (in radians)

        initial_pose.orientation.x = orientation.x();
        initial_pose.orientation.y = orientation.y();
        initial_pose.orientation.z = orientation.z();
        initial_pose.orientation.w = orientation.w();

        if(agent->type_ == "Circle") {

            auto circle_agent_ptr = std::dynamic_pointer_cast<CircleAgent<2> >(agent);
            std::string file_path2 = createCircleAgent(std::string("Circle_")+std::to_string(agent->id_), 
                                                       circle_agent_ptr->radius_*reso, .1, color);

            spawnAgentGazebo(file_path2, std::string("Circle_")+std::to_string(agent->id_), initial_pose, client, node);

        } else if(agent->type_ == "Block_2D") {

            auto block_agent_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent);
            std::string file_path3 = createBlockAgent(std::string("Block_2D_")+std::to_string(agent->id_), 
                                                      {block_agent_ptr->min_pt_[0]*reso, block_agent_ptr->min_pt_[1]*reso},
                                                      {block_agent_ptr->max_pt_[0]*reso, block_agent_ptr->max_pt_[1]*reso}, .1, color);

            spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, client, node);

        }
    }
    // add gridmap to gazebo， as object is too slow, should add them as picture
    // for(int x=0; x<dim[0]; x++) {
    //     for(int y=0; y<dim[1]; y++) {
    //         if(is_occupied({x, y})) {
    //             // draw occupied grid
    //             int id = x + y * dim[0];
    //             std::string file_path3 = createBlockAgent(std::string("grid_")+std::to_string(id), 
    //                                                     {-0.5*reso, -0.5*reso},
    //                                                     { 0.5*reso,  0.5*reso}, .1, cv::Vec3b::all(150));

    //             initial_pose.position.x = global_offset_x + reso*x;  // 设置模型初始位置
    //             initial_pose.position.y = global_offset_y + reso*y;

    //             spawnAgentGazebo(file_path3, std::string("grid_")+std::to_string(id), initial_pose, client, node);
    //         }
    //     }
    // }
    auto layered_paths = layeredLargeAgentMAPF<2>(instances.front().second,
                                                  instances.front().first,
                                                  dim, is_occupied,
                                                  CBS::LargeAgentCBS_func<2>,
                                                  grid_visit_count_table,
                                                  60, decomposer_ptr,
                                                  false);

    auto end_t = clock();

    double time_cost = ((double)end_t-start_t)/CLOCKS_PER_SEC;

    std::cout << (layered_paths.size() == instances.front().first.size() ? "success" : "failed")
              << " layered large agent mapf in " << time_cost << "s " << std::endl;
    std::cout << std::endl;

    // TODO: set all agent to target
    for (int id=0; id<instances.front().first.size(); id++) {
        const auto& agent = instances.front().first[id];
        const auto& target_pt = instances.front().second[id].second.pt_; 
        auto color = COLOR_TABLE[id%30];
        double target_x = global_offset_x + reso*target_pt[0], 
               target_y = global_offset_y + reso*target_pt[1], 
               target_theta = orientToRadius(instances.front().second[id].second.orient_);
        if(agent->type_ == "Circle") {

            // auto circle_agent_ptr = std::dynamic_pointer_cast<CircleAgent<2> >(agent);
            // std::string file_path2 = createCircleAgent(std::string("Circle_")+std::to_string(agent->id_), 
            //                                            circle_agent_ptr->radius_*reso, .1, color);

            // initial_pose.position.x = global_offset_x;  // 设置模型初始位置
            // initial_pose.position.y = global_offset_y;
            // initial_pose.position.z = 1.;

            setModelPose(std::string("Circle_")+std::to_string(agent->id_), target_x, target_y, 1.0, target_theta, set_pose_clinet, node);

        } else if(agent->type_ == "Block_2D") {

            // auto block_agent_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent);
            // std::string file_path3 = createBlockAgent(std::string("Block_2D_")+std::to_string(agent->id_), 
            //                                           {block_agent_ptr->min_pt_[0]*reso, block_agent_ptr->min_pt_[1]*reso},
            //                                           {block_agent_ptr->max_pt_[0]*reso, block_agent_ptr->max_pt_[1]*reso}, .1, color);

            // initial_pose.position.x = global_offset_x;  // 设置模型初始位置
            // initial_pose.position.y = global_offset_y;
            // initial_pose.position.z = 1.;

            // spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, client, node);
            
            setModelPose(std::string("Block_2D_")+std::to_string(agent->id_), target_x, target_y, 1.0, target_theta, set_pose_clinet, node);

        }
        // break;
    }


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

int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    // 创建服务客户端
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
                node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // 等待服务可用, ok
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "等待 /spawn_entity 服务...");
    }

    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_set_pose =
                node->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state"); // /gazebo/set_entity_state

                
    // 等待服务可用, 不ok
    while (!client_set_pose->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "等待 /gazebo/set_entity_state 服务...");
    }

    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"), client, client_set_pose, node);

    return 0;
}