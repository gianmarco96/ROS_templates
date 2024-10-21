# NOT TESTED
Swapping tools or end-effectors in ROS 2 using MoveIt involves updating the robot's kinematic model and planning scene to reflect the change in hardware. This allows the robot to plan and execute motions accurately with the new tool. Below is a comprehensive guide on how to achieve tool swapping in MoveIt for ROS 2.

## Overview

The process involves:

1. **Modeling the Tool Changer and Tools**: Define the tool changer and all possible tools in your robot's Unified Robot Description Format (URDF) or Simulation Description Format (SDF) file.

2. **Dynamic Attachment/Detachment**: Programmatically attach or detach tools at runtime using MoveIt's planning scene and kinematic capabilities.

3. **Updating the Kinematic Chain**: Ensure that the robot's kinematic chain reflects the current tool configuration for accurate motion planning.

4. **Collision Management**: Update the collision objects in the planning scene to prevent unwanted collisions during motion planning.

## Step-by-Step Guide

### 1. Model the Tool Changer and Tools in URDF

- **Tool Changer**: Define a link and a joint in your URDF that represents the tool changer mechanism on your robot. This joint will typically be a fixed or floating joint where tools can be attached.

    ```xml
    <!-- Tool changer link -->
    <link name="tool_changer"/>
    
    <!-- Fixed joint connecting the robot arm to the tool changer -->
    <joint name="arm_to_tool_changer" type="fixed">
      <parent link="robot_arm_end_link"/>
      <child link="tool_changer"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

- **Tools**: For each tool, create separate URDF files or include them within the main URDF with unique link and joint names.

    ```xml
    <!-- Tool link -->
    <link name="gripper_tool"/>
    
    <!-- Joint connecting the tool changer to the tool -->
    <joint name="tool_changer_to_gripper" type="fixed">
      <parent link="tool_changer"/>
      <child link="gripper_tool"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

### 2. Load Tools as Separate Robot Descriptions (Optional)

If tools have complex kinematics or sensors, consider loading them as separate robot descriptions and merge them at runtime.

- **ROS 2 Parameter Server**: Use the parameter server to store multiple robot descriptions and switch between them.

- **Robot State Publisher**: Run multiple instances if necessary, one for the base robot and others for the tools.

### 3. Programmatically Attach/Detach Tools

Use MoveIt's `RobotState` and planning scene interfaces to attach or detach tools at runtime.

- **Attach Tool**:

    ```python
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from moveit_commander import PlanningSceneInterface, RobotCommander
    from geometry_msgs.msg import PoseStamped
    
    # Initialize MoveIt interfaces
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    
    # Define the tool as a collision object
    tool = CollisionObject()
    tool.id = "gripper_tool"
    tool.header.frame_id = "tool_changer"
    
    # Add the tool's geometry (e.g., from a mesh or primitive shape)
    # tool.meshes = [...]
    # tool.mesh_poses = [...]
    # tool.operation = CollisionObject.ADD
    
    # Attach the tool to the robot
    attach = AttachedCollisionObject()
    attach.link_name = "tool_changer"
    attach.object = tool
    scene.attach_object(attach.object.id, "tool_changer", touch_links=["gripper_tool"])
    ```

- **Detach Tool**:

    ```python
    scene.remove_attached_object("tool_changer", name="gripper_tool")
    ```

### 4. Update the Kinematic Chain

Ensure that the robot's kinematic model includes the tool for accurate motion planning.

- **Update Robot State**:

    ```python
    robot_state = robot.get_current_state()
    # Update the robot state to include the attached tool
    robot_state.attach_body(attach)
    ```

- **Set the Robot State in Move Group**:

    ```python
    move_group = moveit_commander.MoveGroupCommander("arm")
    move_group.set_start_state(robot_state)
    ```

### 5. Manage Collision Objects

Update the planning scene to reflect the current state of the robot and environment.

- **Add/Remove Collision Objects**: Use the PlanningSceneInterface to add or remove collision objects representing tools or obstacles.

### 6. Plan and Execute Motions

With the tool attached and the planning scene updated, you can now plan and execute motions as usual.

- **Planning**:

    ```python
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = ...
    pose_goal.pose.position.y = ...
    pose_goal.pose.position.z = ...
    pose_goal.pose.orientation.w = 1.0

    move_group.set_pose_target(pose_goal, end_effector_link="gripper_tool")
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    ```

### 7. Handling Multiple Tools

Repeat the attach/detach process for different tools as needed.

- **Switching Tools**:

    ```python
    # Detach current tool
    scene.remove_attached_object("tool_changer", name=current_tool_id)
    
    # Attach new tool
    attach_new_tool()
    ```

## Additional Considerations

- **Tool Mass and Inertia**: Update the robot's dynamics properties if the tool significantly affects the robot's mass distribution.

- **Sensor Integration**: If the tools have sensors (e.g., force-torque sensors), ensure that their data streams are properly integrated.

- **Real Robot vs. Simulation**: When working with a real robot, ensure that the physical tool changing mechanism is synchronized with the software state.

- **Collision Checking**: Update the allowed collision matrix if certain parts should be allowed to collide (e.g., the tool and tool changer during attachment).

- **Subframes (ROS 2 Feature)**: Use subframes to define precise grasp or tool attachment points within MoveIt, improving accuracy.

    ```python
    # Define a subframe in your URDF or programmatically
    ```

## Example Repository and Tutorials

- **MoveIt Tutorials for ROS 2**: [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)

- **Example Implementation**: Check out [PickNikRobotics/moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials) for examples related to robot states, planning scenes, and attaching objects.

## Conclusion

Swapping tools in ROS 2 using MoveIt requires careful modeling and programmatic control of the robot's kinematic chain and planning scene. By defining the tool changer and tools in your URDF, and using MoveIt's capabilities to attach and detach tools at runtime, you can effectively manage multiple tools and plan accurate motions for your robotic applications.

---

**Note**: Always ensure compatibility between the URDF models, the MoveIt configuration, and the physical robot hardware to prevent discrepancies between the planned and executed motions.
