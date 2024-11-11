#ifndef FAKE_AGENTS
#define FAKE_AGENTS

#include <cstdio>
//#include <LA-MAPF/algorithm/LA-MAPF/action_dependency_graph.h>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <fstream>
#include <sstream>
#include <string>

#include "common_interfaces.h"

void createCylinderSDFFile(const std::string& model_name, const std::string& file_path, double radius, double height) {
    // SDF 模板，定义一个带圆柱体的模型
    std::string sdf_content = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius1}</radius>
            <length>{height1}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius2}</radius>
            <length>{height2}</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
)";

    // 替换模型名称、半径和高度
    size_t pos = sdf_content.find("{model_name}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{model_name}").length(), model_name);
    }
    pos = sdf_content.find("{radius1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{radius1}").length(), std::to_string(radius));
    }
    pos = sdf_content.find("{height1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height1}").length(), std::to_string(height));
    }
    pos = sdf_content.find("{radius2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{radius2}").length(), std::to_string(radius));
    }
    pos = sdf_content.find("{height2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height2}").length(), std::to_string(height));
    }

    // std::cout << " sdf_content = " << sdf_content << std::endl;

    // 将SDF内容写入文件
    std::ofstream sdf_file(file_path);
    if (sdf_file.is_open()) {
        sdf_file << sdf_content;
        sdf_file.close();
        std::cout << "SDF file created with cylinder at: " << file_path << std::endl;
    } else {
        std::cerr << "Failed to create SDF file at: " << file_path << std::endl;
    }
}

void createBlockSDFFile(const std::string& model_name, const std::string& file_path, 
                       const freeNav::Pointf<2>& pt_min, const freeNav::Pointf<2>& pt_max,
                       double height) {

    double length = pt_max[0] - pt_min[0];
    double width  = pt_max[1] - pt_min[1];
    double x      = (pt_max[0] + pt_min[0])/2;
    // SDF 模板，定义一个带圆柱体的模型
    std::string sdf_content = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <link name="base_link">
      <pose>{x} 0 0 0 0 0</pose>
      <static>true</static>
      <collision name="collision">
        <geometry>
          <box>
            <size>{length1} {width1} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length2} {width2} {height}</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
)";

    // 替换模型名称、半径和高度
    size_t pos = sdf_content.find("{model_name}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{model_name}").length(), model_name);
    }
    pos = sdf_content.find("{x}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{x}").length(), std::to_string(x));
    }

    pos = sdf_content.find("{length1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{length1}").length(), std::to_string(length));
    }
    pos = sdf_content.find("{width1}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{width1}").length(), std::to_string(width));
    }
    pos = sdf_content.find("{length2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{length2}").length(), std::to_string(length));
    }
    pos = sdf_content.find("{width2}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{width2}").length(), std::to_string(width));
    }
    pos = sdf_content.find("{height}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height}").length(), std::to_string(height));
    }
    pos = sdf_content.find("{height}");
    if (pos != std::string::npos) {
        sdf_content.replace(pos, std::string("{height}").length(), std::to_string(height));
    }
    // std::cout << " sdf_content = " << sdf_content << std::endl;

    // 将SDF内容写入文件
    std::ofstream sdf_file(file_path);
    if (sdf_file.is_open()) {
        sdf_file << sdf_content;
        sdf_file.close();
        std::cout << "SDF file created with cylinder at: " << file_path << std::endl;
    } else {
        std::cerr << "Failed to create SDF file at: " << file_path << std::endl;
    }
}

std::string loadSDFFile(const std::string& file_path) {
    std::ifstream sdf_file(file_path);
    std::stringstream sdf_buffer;
    sdf_buffer << sdf_file.rdbuf();
    return sdf_buffer.str();
}

std::string createCircleAgent(const std::string& model_name, double radius, double height) {
    std::string file_path = "/tmp/fake_large_agents/" + model_name + ".sdf";
    createCylinderSDFFile(model_name, file_path, radius, height);
    return file_path;
}

std::string createBlockAgent(const std::string& model_name,
                             const freeNav::Pointf<2>& pt_min,
                             const freeNav::Pointf<2>& pt_max,
                             double height) {
    std::string file_path = "/tmp/fake_large_agents/" + model_name + ".sdf";
    createBlockSDFFile(model_name, file_path, pt_min, pt_max, height);
    return file_path;
}

typedef rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr SpawnClientPtr;

bool spawnAgentGazebo(const std::string& file_path, const std::string model_name, 
                      const geometry_msgs::msg::Pose& initial_pose, 
                      const SpawnClientPtr& client, const rclcpp::Node::SharedPtr& node
                      ) {


    // 读取SDF文件内容
    std::string model_xml = loadSDFFile(file_path);
    // 创建SpawnEntity请求
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;              // 设置模型名称
    request->xml = model_xml;                // SDF文件内容
    request->robot_namespace = "";           // 机器人命名空间，可留空
    request->initial_pose = initial_pose;    // 设置模型初始位置
    request->reference_frame = "world";      // 设置参考坐标系

    // 等待服务启动
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for /spawn_entity service to be available...");
    }

    // 发送请求并等待响应
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Successfully spawned model: %s", model_name.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to spawn model: %s", model_name.c_str());
        return false;
    }
    return true;
}

#endif