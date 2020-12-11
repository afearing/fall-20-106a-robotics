- http://wiki.ros.org/aruco
  - ROS Aruco page. Doesn't seem too useful
- http://www.uco.es/investiga/grupos/ava/node/26
  - Main Aruco page
- https://youtu.be/WDhIaVOUwsk
  - Video on setting up AR tags
- https://artisticrender.com/how-to-add-a-texture-to-an-object-in-blender/
  - Tutorial on UV texture mapping in Blender
- http://wiki.ros.org/aruco_detect
  - Maybe use aruco instead of alvar? better documentation and i know how.
# steps
## Aruco tags
1. Install aruco detect package (not in use yet)
   - http://wiki.ros.org/aruco_detect
1. Generate aruco tags
   - https://chev.me/arucogen/
1. Use Blender to apply texture to 3D object and export to .dae file. You might have to set the axes to align with Gazebo defaults (Gazebo uses a right-hand coordinate system where +Z is up (vertical), +X is forward (into the screen), and +Y is to the left)
   - https://artisticrender.com/how-to-add-a-texture-to-an-object-in-blender/
## commands
```
rosrun tf tf_echo [reference_frame] [target_frame]

./baxter.sh sim
roslaunch arucobaxter_gazebo arucobaxter_bothmodels.launch 
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=
true left_electric_gripper:=true
rosrun tf tf_echo base right_hand
rostopic echo /fiducial_transforms
```
