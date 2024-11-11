#include "fake_agents.h"

int main(int argc, char ** argv) {

    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("spawn_sdf_model");

    // 创建服务客户端
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
                node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = 0.0;  // 设置模型初始位置
    initial_pose.position.y = 0.0;
    initial_pose.position.z = 0.5;
    initial_pose.orientation.w = 1.0;

    // // 生成带圆柱体的SDF文件, 圆柱体中心initial pose参考点
    // std::string file_path = createCircleAgent(std::string("cylinder_robot1"), .3, .7);
    // spawnAgentGazebo(file_path, std::string("cylinder_robot1"), initial_pose, client, node);

    // // 生成带圆柱体的SDF文件
    // initial_pose.position.x = 1.0;  // 设置模型初始位置
    // initial_pose.position.y = 2.0;
    // initial_pose.position.z = 0.5;

    // std::string file_path2 = createCircleAgent(std::string("cylinder_robot2"), .5, .3);
    // spawnAgentGazebo(file_path2, std::string("cylinder_robot2"), initial_pose, client, node);


    std::string file_path3 = createBlockAgent(std::string("block_robot"), {0, -.5}, {1, .5}, 1.0);
    spawnAgentGazebo(file_path3, std::string("block_robot"), initial_pose, client, node);

    // 关闭ROS 2
    rclcpp::shutdown();

    return 0;

}