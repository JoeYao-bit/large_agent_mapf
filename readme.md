 
更新launch后需要重新运行 colcon build 以更新位于install的launch复制文件，单独更新src节点内的launch无效

运行下述代码启动gazebo以及相关world文件

```
ros2 launch large_agent_visualize gazebo_launch.py 
```

LA-MAPF 和 freeNav-base的头文件安装在/usr/local/include目录，分别来自freeNav和LayeredMAPF

启动planner： ros2 run large_agent_mapf lamapf_planner_node 

libcanvas_ex.a liblamapf_alg_ex.a 分别是freeNav和LayeredMAPF编译得到的静态库，安装在/usr/local/lib

11-11：同样的输入参数和代码，运行速度比在Clion慢了好几倍，不知道为什么

colcon build --cmake-args -DCMAKE_CXX_FLAGS="-O3"

通过设置编译时O3优化，运行速度大大提高了

启动rviz： ros2 run rviz2 rviz2

启动fake agent：ros2 run large_agent_mapf fake_agents_node


使用ROS2 命令行生成一个机器人（运行成功）
ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: "sdf_ball", xml: "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual></link></model></sdf>"}'


[gazebo-1] Error [Param.cc:449] Invalid argument. Unable to set value [{radius} ] for key[radius].
[gazebo-1] Error [parser_urdf.cc:3267] Unable to call parseURDF on robot model
[gazebo-1] Error [parser.cc:488] parse as old deprecated model file failed.
[gazebo-1] Error Code 8 Msg: Error reading element <radius>
[gazebo-1] Error Code 8 Msg: Error reading element <cylinder>
[gazebo-1] Error Code 8 Msg: Error reading element <geometry>
[gazebo-1] Error Code 8 Msg: Error reading element <collision>
[gazebo-1] Error Code 8 Msg: Error reading element <link>
[gazebo-1] Error Code 8 Msg: Error reading element <model>
[gazebo-1] Error Code 8 Msg: Error reading element <sdf>


<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="cylinder_robot">
    <link name="base_link">
      <pose>0 0 0.5 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>2.300000</radius>
            <length>1.000000</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>

    // 替换模型名称、半径和高度 需要多次进行，每一个都设置一遍
