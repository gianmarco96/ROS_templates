****************************************
Unified Robot Description Format URDF_
****************************************
.. note::
  Refer to URDF-Tutorials_ and clone the URDF-Tutorials-repo_ if you follow the official tutorial ::

    git clone https://github.com/ros/urdf_tutorial.git

In this chapter we will create a model of Epson SCARA Robot.

In the workspace in the ``src`` directory create a folder called Robots, where we will put all robot description packages. ::

  // navigate to src
  mkdir Robots

  catkin_create_pkg epson_g3_description geometry_msgs urdf rviz xacro

  cd epson_g3_description

  mkdir urdf scripts rviz


Back to the workspace and compile the packages.

Launch file
=============
The following launch file is taken from URDF-Tutorials_. It have different parameters that allow it to execute different robot models.

.. code-block:: xml
  :caption: display.launch from urdf_tutorial

  <launch>
    <arg name="model" default="$(find urdf_tutorial)/urdf/01-myfirst.urdf"/>
    <arg name="gui" default="true" />
    <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />

    <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />
    <param name="use_gui" value="$(arg gui)"/>

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
  </launch>

It execute 3 nodes: rviz_, joint_state_publisher_, state_publisher_. In the launch file are present 2 parameters ``robot_description`` and ``use_gui`` that are need by the two nodes``joint_state_publisher`` and ``state_publisher``. There are also 3 arguments with default values, one of them is full name of the urdf file.

The node ``joint_state_publisher`` read the urdf file from the parameter ``robot_description`` finds all of the non-fixed joints and publishes a ``sensor_msgs/JointState`` message with all those joints defined on the topic ``/joint_states``. We set by default the parameter ``use_gui`` to true, so when the node is executed it will show a window/widget with sliders that let us control the robot joints.

The node ``state_publisher`` uses the URDF specified by the parameter ``robot_description`` and the joint positions from the topic ``/joint_states`` to calculate the forward kinematics of the robot and publish the results via ``tf``.

Launch the ``display.launch`` file ::

  roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf

Or independently from the working direcory ::

  roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'

In this case the file name is the same of the default value. So in this case it can be omitted ::

  roslaunch urdf_tutorial display.launch

Rviz files can be deleted from the launch file if you don't have them. They can be created from rviz later. If there is no rviz file, Rviz will not show the frames neither the robot neither select the right ``Fixed Frame``.

We will modify launch file from the tutorial in the followingway:

.. code-block:: xml
  :caption: display.launch

  <launch>

    <arg name="model" default="$(find epson_g3_description)/urdf/scara.urdf"/>
    <arg name="gui" default="true" />

    <param name="robot_description" textfile="$(find epson_g3_description)/urdf/scara.urdf" />
    <param name="use_gui" value="$(arg gui)"/>

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
    <node name="rviz" pkg="rviz" type="rviz" />

  </launch>

:download:`Download display.launch <../../../code/Robots/epson_g3_description/launch/display.launch>`

Basically I change the package name, the urdf default name and delete the reference for rviz configuration file. We will back later to rviz configuration files.

I create also a scripts folder where I will create some bash file to help me executing nodes. Simply in the bash script launch.sh there is: ::

   roslaunch epson_g3_description display.launch model:='$(find epson_g3_description)/urdf/scara.urdf'


URDF basics
============

URDF is an xml file that describe the geometry of a robot. URDF is a tree structure with one root link. The measuring units are meters and radians.

Robot
-------
A robot is composed mainly from links, and joints.

.. code-block:: xml

  <robot name="robot_name">
    <link>  </link>
    <link>  </link>

    <joint>  </joint>
    <joint>  </joint>
  </robot>

Link
------
The link element describes a rigid body with an inertia, visual features, and collision properties.

.. _figLink:
.. figure:: images/link.png
    :align: center
    :figwidth: 400px

The main components of ``link`` tag are as follow:

.. code-block:: xml

  <link name="link_name">

    <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <cylinder length="0.6" radius="0.2"/>
        </geometry>
    </visual>

    <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="0.6" radius="0.2"/>
        </geometry>
    </collision>

    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="1"/>
        <inertia
            ixx="1.0" ixy="0.0" ixz="0.0"
            iyy="1.0" iyz="0.0"
            izz="1.0"/>
    </inertial>
  </link>

The ``visual`` tag specifies the shape of the object (``box``, ``cylinder``, ``sphere``, ``mesh``, etc.) for visualization purposes.
Its ``origin`` is the reference frame of the visual element with respect to the reference frame of the link (The reference frame of the link is its joint).

The ``collision`` can be the same as visual, or its geometry a little bit bigger. Its ``origin`` is the reference frame of the collision element, relative to the reference frame of the link.

The ``inertial`` tag is need if the model is loaded in a simulator with physics engine. Its ``origin`` is the pose of the inertial reference frame, relative to the link reference frame. The origin of the inertial reference frame needs to be at the center of gravity. The axes of the inertial reference frame do not need to be aligned with the principal axes of the inertia.

Joint
------
The joint describe the relative motion between two links. It can be ``revolute``, ``continuous``, ``prismatic``, ``fixed``, ``floating``, ``planar``.

.. _figJoint:
.. figure:: images/joint.png
   :align: center
   :figwidth: 400px

Basic properties of a joint tag:

.. code-block:: xml

  <joint name="joint_name" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>

    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>

  </joint>

The ``origin`` is the transform from the parent link to the child link. The joint is located at the origin of the child link. So the origin is the relative position of the child frame respect to the parent frame.

The joint ``axis`` specified in the joint frame. This is the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. The ``axis`` is specified in the joint frame of reference. Fixed and floating joints do not use the ``axis`` field.

The ``joint`` have other properties as ``dynamics``, ``limit``, etc. ``Limits`` are in radians for revolute joints and meters for prismatic joints and are omitted if the joint is continuous or fixed.

The following image show the relationship between two joints.

.. _figjoint_vis:
.. figure:: images/joint_vis.png
   :align: center
   :figwidth: 400px

The previous image is produced by the following URDF model. Note that there is no visual aspect of the links. Only joints are defined.

.. code-block:: xml

  <?xml version="1.0"?>
  <robot name="origins">

    <link name="base_link">
    </link>

    <link name="right_leg">
    </link>

    <joint name="base_to_right_leg" type="fixed">
      <parent link="base_link"/>
      <child link="right_leg"/>
      <origin xyz="0 -0.22 0.25"/>
    </joint>

  </robot>

Tansmission
-------------
Transmissions link actuators to joints and represents their mechanical coupling. The transmission element is an extension to the URDF robot description model that is used to describe the relationship between an actuator and a joint. This allows one to model concepts such as gear ratios and parallel linkages. A transmission transforms efforts/flow variables such that their product - power - remains constant. Multiple actuators may be linked to multiple joints through complex transmission.

.. code-block:: xml

  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>

    <joint name="joint1">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>

    <actuator name="motor1">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>

  </transmission>

Gazebo
-------
Gazebo can be add to different elements. Refer to the URDF-Tutorials_ and Gazebo_ tutorials. In order to simulate the model correctly in Gazebo_ at least he ``inertia`` and ``transmission`` tags for all links should be defined.

Other properties
-----------------

Refer to URDF-XML_ and URDF-Tutorials_ for more information.

Complete Robot model example
==============================

Joint and frame definition
---------------------------

We will define the 3D model of Epson Scara robot G3-251s. This robot have 4 links and 3 joints.
On link is fixed, ``base_link``. Two links rotate, we will call them link1 and link2. And the last link, link3, has translation motion.

.. _figEpsonG3:
.. figure:: images/scara/g3-251s.jpg
   :align: center
   :figwidth: 400px

The mechanical drawing is shown below:

.. _figEpsonMech1:
.. figure:: images/scara/mech1.png
   :align: center
   :figwidth: 400px

.. _figEpsonMech2:
.. figure:: images/scara/mech2.png
  :align: center
  :figwidth: 400px

.. _figEpsonMech3:
.. figure:: images/scara/mech3.png
   :align: center
   :figwidth: 400px

We will define first the links without visual aspect. Then we define the joints. The relative positions of the joints are taken from the previous images.
As we can see, we have 2 ``revolute`` joints and one ``prismatic``.

First we define the links, without the visual aspect:

.. code-block:: xml

  <link name="base_link">
  </link>

  <link name="link_1">
  </link>

  <link name="link_2">
  </link>

  <link name="link_3">
  </link>

We define the ``joint_1`` between the base and link1. We can define it where we want along the rotate axis. But we will define it in the contact between link1 and the base.
In zero position, all links open, the robot lie on the x axis of the base link. Joint1 has on offset in z of 129mm, so 0.129 m . The joint is define on the child link. Link1 rotate arround the z axis of joint1.

.. code-block:: xml

  <joint name="link1_to_base" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.129"/>
    <axis xyz="0 0 1" />
    <limit effort="300" velocity="0.1" lower="-3.14" upper="3.14"/>
  </joint>

In the same way we define joint2. As you can see the dimension ``a`` in the image differ from model to model. We will see later how we cam parameterize the robot model. For now we take the value for the model G3-251S, that is 120 mm.

.. code-block:: xml

  <joint name="link2_to_link1" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.120 0 0"/>
    <axis xyz="0 0 1" />
    <limit effort="300" velocity="0.1" lower="-3.14" upper="3.14"/>
  </joint>

The last joint, is prismatic. Link_3 slide along axis z. The stroke is 150mm.

.. code-block:: xml

  <joint name="link3_to_link2" type="prismatic">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0.130 0 0"/>
    <axis xyz="0 0 1" />
    <limit effort="300" velocity="0.1" lower="0" upper="0.150"/>
  </joint>

We can use the command ``check_urdf`` to check if the urdf file have errors: ::

  check_urdf scara.urdf

Run the launch file or run the script. ::

  roslaunch epson_g3_description display.launch

.. _figRvizScara:
.. figure:: images/scara/scara.gif
   :align: center
   :figwidth: 700px

:download:`Download scara.urdf <../../../code/Robots/epson_g3_description/urdf/scara.urdf>`

Check URDF model
-------------------
Navigato to urdf direcory then: ::

  check_urdf scara.urdf

If there are no errors, you will see the parents childs tree that define the robot.

The command ::

  urdf_to_graphiz scara.urdf

will create 2 files: ``scara.gv`` and ``scara.pdf``.

Visual aspect and mesh
------------------------
Espson provide CAD files in step and solidworks format. Stp can be opend on linux using FreeCad.
We need to select every link, convert it individually in stp. Pay attention to change the measuring unit. If you convert it in mm, you will see a model 1000 tmes bigger than the real robt in rviz. URDF use meter as unit. So in FreeCad change the unit in meter then export to stp. After that open the stp file and convert it to dae. In order to convert to dae the library ``pycollada`` should be installed.

Another way to convert to ``dae``, is to convert the links in ``vrml`` using FreeCad, then open the ``wrml`` in MeshLab then convert to ``dae``.

.. figure:: images/freecad_dae.gif
   :align: center
   :figwidth: 700px

.. _URDF: wiki.ros.org/urdf
.. _URDF-XML: http://wiki.ros.org/urdf/XML
.. _URDF-Tutorials: http://wiki.ros.org/urdf/Tutorials
.. _URDF-Repo: https://github.com/ros/urdf.git
.. _URDF-Tutorials-repo: https://github.com/ros/urdf_tutorial

.. _joint_state_publisher: http://wiki.ros.org/joint_state_publisher
.. _robot_state_publisher: http://wiki.ros.org/robot_state_publisher
.. _state_publisher: http://wiki.ros.org/robot_state_publisher/Tutorials
.. _rviz: http://wiki.ros.org/rviz

.. _Gazebo: http://www.gazebosim.org/

# Verification
A command line tool check_urdf attempts to parse a file as a URDF description, and either prints a description of the resulting kinematic chain, or an error message.

For example, to run this tool on the pr2 urdf, first create the urdf file by running:
```
rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro -o /tmp/pr2.urdf
```

Then run the check by running:

Note: You may need to run sudo apt-get install liburdfdom-tools.
```
check_urdf pr2.urdf
```

Then run the check by running:
```
rosrun urdfdom check_urdf /tmp/pr2.urdf
```
and you should see something resembling:
```
robot name is: pr2
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  base_laser_link
        child(2):  bl_caster_rotation_link
            child(1):  bl_caster_l_wheel_link
            child(2):  bl_caster_r_wheel_link
        child(3):  br_caster_rotation_link
            child(1):  br_caster_l_wheel_link
            child(2):  br_caster_r_wheel_link
        child(4):  fl_caster_rotation_link
            child(1):  fl_caster_l_wheel_link
            child(2):  fl_caster_r_wheel_link
        child(5):  fr_caster_rotation_link
            child(1):  fr_caster_l_wheel_link
            child(2):  fr_caster_r_wheel_link
        child(6):  torso_lift_link
            child(1):  head_pan_link
                child(1):  head_tilt_link
                    child(1):  head_plate_frame
                        child(1):  sensor_mount_link
                            child(1):  double_stereo_link
                                child(1):  narrow_stereo_link
 ...
 ```
# Visualization
To get a graphviz diagram of your urdf file, do the following:


```
urdf_to_graphiz pr2.urdf
```

