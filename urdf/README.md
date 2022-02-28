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

