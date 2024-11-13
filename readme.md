 
更新launch后需要重新运行 colcon build 以更新位于install的launch复制文件，单独更新src节点内的launch无效

运行下述代码启动gazebo以及相关world文件

```
ros2 launch large_agent_mapf gazebo_launch.py 
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

通过代码构造SDF，并存放在/tmp目录下

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

gazebo world文件地址：/home/yaozhuo/software/gazebo/warehouse.world


    <?xml version ?>  xml的版本
    <sdf version>  sdf的版本，和config里<sdf>的版本要一样呀
    <model name> 模型的名字
    <pose> 在世界中的位置 x y z pitch yaw roll
    <static> 选择模型是否固定
    <link>链接 包含模型的一个主体的物理属性，尽量减少模型中链接数量以提高性能和稳定
    <collision>: 用于碰撞检查，一个link可以有多个碰撞元素
    <geometry> 物体
    <box> | <sphere> | <cylinder>形状名字
    <size> x y z长度 | <radius>半径 | <radius> & <length>
    <surface> 平面
    <friction>设置地面摩擦力
    <ode> <mu> <slip>
    <visual>: 可视化
    <geometry> 几何形状
    <box>形状名字
    <size> x y z长度
    <inertial>: 惯性元素，描述了link的动态特性，例如质量和转动惯量矩阵
    <mass> 质量
    <inertia> ！！！注意这两单词不一样呀
    <sensor>: 从world收集数据用于plugin
    <light>: 光源
    <joint>关节 关节连接两个link，用于旋转轴和关节限制等
    <plugin>插件  用于控制模型

删除collision可以忽略物理模型

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius1}</radius>
            <length>{height1}</length>
          </cylinder>
        </geometry>
      </collision>

这是一个自问自答的问题记录。

在使用ros2 control CLI时遇到的RTPS报错，提示无法连接服务。

$ ros2 control list_hardware_interfaces
2022-03-27 12:15:30.282 [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7412: open_and_lock_file failed -> Function open_port_internal
Could not contact service /controller_manager/list_hardware_interfaces

造成该问题的原因是找不到和这个服务通信的数据类型。

可以采用下面的命令安装下，重新运行即可。其中foxy可以替换为你自己的ros2版本。
sudo apt install ros-foxy-controller-manager*

/----------------------------------------------/

libgazebo_ros_state（set entity state服务需要），不是系统插件，需要在.world中注册


<sdf version='1.7'>
  <world name='default'>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
        <argument>model_states:=model_states</argument>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>
...

参考链接： https://robotics.stackexchange.com/questions/96506/how-to-use-gazebo-plugins-found-in-gazebo-ros-ros2-foxy-gazebo11

修改之后仍然报错，如下，但可以通过set_entity_state服务设置机器人位姿了
[gazebo-1] [Err] [gazebo_shared.cc:46] System is attempting to load a plugin, but detected an incorrect plugin type. Plugin filename[libgazebo_ros_state.so].

/usr/local/inlcude，/usr/local/lib 本地安装的头文件和库地址

/usr/inlcude，/usr/lib 下载安装的头文件和库地址

TODO： 创建一个服务，如果已经规划完路径，则从头开始演示执行过程