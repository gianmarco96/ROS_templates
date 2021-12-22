# ROS_templates
This repo is a collection of the main ROS templates 
## Create srv file first (e.g. ServiceOne.srv)
```
int64 num1
int64 num2
---
bool success
```
## Build service C++
```
<exec_depend>message_runtime</exec_depend>
 <build_depend>message_generation</build_depend>
```
should exist in package xml


& in CMakeFile
```
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
...
message_generation



add_service_files(
	FILES
	Nam
  )
  
generate_messages(
   DEPENDENCIES
   geometry_msgs#   iiwa_msgs#   moveit_msgs#   sensor_msgs#   std_msgs#   tf2_msgs#   trajectory_msgs
 )
 
```


##Building a node in CMake
```
add_executable(${PROJECT_NAME}_node src/readState.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


```
