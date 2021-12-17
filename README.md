# ROS_templates
This repo is a collection of the main ROS templates 
## Create srv file first (e.g. ServiceOne.srv)
```int64 num1
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
```
