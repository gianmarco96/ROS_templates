# ROS_templates
This repo is a collection of the main ROS templates 

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
```
