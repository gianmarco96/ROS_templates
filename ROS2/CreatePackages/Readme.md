# Python
## Create the package
```
ros2 pkg create --build-type ament_python <my_package> --dependencies <rclpy> 
```
This creates the following structure
```
ros2_ws
  build
  install
  log
  src
    my_package (folder)
      my_package (folder)
        .py files are stored here
      resource (folder)
      test (folder)
      package.xml
      setup.cfg
      setup.py
```  
## Building
[] -> means optional commands
```
colcon build [--packages-select <package-name>]
```
Sourcing is inside install folder
```
source ~/rosw_ws/install/setup.bash
```
Everything needs to be built even launch, xml, and python files. There is an expection with the symlink install argument but not quite there yet
## See available (installed) packages
```
ros2 pkg list
```
