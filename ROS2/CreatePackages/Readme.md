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
      setup.py (analogous to the C++ CMakeLists.txt)
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
## Launch files
Launch files need to be created with the extension .launch.py (although I believe this can be changed in the setup.py file)

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='simple',
            output='screen'),
    ])

```
If you want to add a new executable this needs to be added to the setup file (as well as you need to redirect to the folder where the launch files are stored)
```
from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '<executable_name_to_use_in_launch> = <package_name>.<script_name>:main'

        ],
    },
)
```
executable name usually same as the executable without .py ext.
The data_file line installs launch files inside the ~/ros2_ws/install/my_package/share/my_package/launch folder

The setup.cfg states where to install executables. e.g. of setup.cgf
```
In [ ]:
[develop]
script_dir=$base/lib/my_package
[install]
install_scripts=$base/lib/my_package
```
