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

    auto instances = deserializer.getTestInstance({40}, 1);


    // add instance to gazebo
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;  // 设置模型初始位置
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.5;
    initial_pose.orientation.w = 1.0;
    for (int id=0; id<instances.front().first.size(); id++) {
        const auto& agent = instances.front().first[id];
        const auto& start_pt = instances.front().second[id].first.pt_; 
        if(agent->type_ == "Circle") {

            auto circle_agent_ptr = std::dynamic_pointer_cast<CircleAgent<2> >(agent);
            std::string file_path2 = createCircleAgent(std::string("Circle_")+std::to_string(agent->id_), 
                                                       circle_agent_ptr->radius_*0.05, .4);

            initial_pose.position.x = 0.05*start_pt[0];  // 设置模型初始位置
            initial_pose.position.y = 0.05*start_pt[1];

            spawnAgentGazebo(file_path2, std::string("Circle_")+std::to_string(agent->id_), initial_pose, client, node);

        } else if(agent->type_ == "Block_2D") {

            auto block_agent_ptr = std::dynamic_pointer_cast<BlockAgent_2D>(agent);
            std::string file_path3 = createBlockAgent(std::string("Block_2D_")+std::to_string(agent->id_), 
                                                      {block_agent_ptr->min_pt_[0]*0.05, block_agent_ptr->min_pt_[1]*0.05},
                                                      {block_agent_ptr->max_pt_[0]*0.05, block_agent_ptr->max_pt_[1]*0.05}, .3);

            initial_pose.position.x = 0.05*start_pt[0];  // 设置模型初始位置
            initial_pose.position.y = 0.05*start_pt[1];

            spawnAgentGazebo(file_path3, std::string("Block_2D_")+std::to_string(agent->id_), initial_pose, client, node);

        }
    }

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

    InstanceVisualization(instances.front().first, decomposer_ptr->getAllPoses(),
                          instances.front().second, layered_paths, grid_visit_count_table);
}

int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    // 创建服务客户端
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
                node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    layeredLargeAgentMAPFTest(map_test_config.at("la_ins_path"), client, node);

    return 0;
}