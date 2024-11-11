cmake_minimum_required(VERSION 3.5)
project(large_agent_mapf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

# find dependencies 
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(include 
                    include/large_agent_mapf
                    )

set(EXTERNAL_LIBS
        libcanvas_ex.a
        liblamapf_alg_ex.a
        pthread
        ${OpenCV_LIBS}
        )

set(dependencies gazebo_msgs rclcpp)        

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(my_node src/my_node.cpp)
# target_include_directories(my_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_link_libraries(my_node ${EXTERNAL_LIBS} )
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

add_executable(fake_agents_node src/fake_agents.cpp)
# target_include_directories(fake_agents_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_link_libraries(fake_agents_node ${EXTERNAL_LIBS} )
install(TARGETS fake_agents_node DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(fake_agents_node
  ${dependencies}
)

add_executable(lamapf_planner_node src/lamapf_planner.cpp)
# target_include_directories(lamapf_planner_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_link_libraries(lamapf_planner_node ${EXTERNAL_LIBS} )
install(TARGETS lamapf_planner_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
