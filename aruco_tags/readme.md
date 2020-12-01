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
## Gazebo worlds and launch
1. Follow this tutorial make a `.launch` file that will spawn things in the world
   - http://gazebosim.org/tutorials?tut=ros_roslaunch
1. create a package for the world launch
   - `catkin_create_pkg arucobaxter_gazebo rospy roscpp std_msgs geometry_msgs gazebo_ros`
1. Setup the Baxter URDF file
   - There are a couple ways to do this with varying effectiveness
   1. Use the xacro included in the `~/rethink_ws/src/baxter_common/baxter_description/urdf`. This is done in arucobaxter_xacro.launch. It puts Baxter in the world, but on its side a few meters from the origin.
   1. Convert the xacro to a URDF using the `.sh` file in `~/rethink_ws/src/baxter_common/baxter_description/urdf`. This doesn't seem to work. Some error with legacy something. `arucobaxter_lab5urdf.launch`
   1. Use lab 5's URDF for Baxter. Can use the one in `~/rethink_ws/src/baxter_common/baxter_description/urdf` or use the one in the package so that it's portable. `arucobaxter_lab5urdf.launch` and `arucobaxter_lab5port.launch` respectively.
1. `roslaunch arucobaxter_gazebo arucobaxter_lab5port.launch`
1. next: figure out how to put the tag in the gazebo sim

1. Follow this guide for how to start Gazebo with a custom world file. IDK how to do gazebo_ros though
   - http://gazebosim.org/tutorials/?tut=import_mesh


## trying out gazebo worlds
1. create a package for the world launch `catkin_create_pkg arucobaxter_gazebo rospy roscpp std_msgs geometry_msgs gazebo_ros`
`roslaunch arucobaxter_gazebo arucobaxter.launch`
- `roslaunch turtlebot_gazebo turtlebot_world.launchworld_file:=$./worlds/room.world` this doesn't seem to work
```rosrun gazebo_ros spawn_model -file `rospack find aruco_planning`/src/baxter.urdf -urdf -z 1 -model baxter```





## alvar (no longer using this)
1. Install `ar_track_alvar`. Use use `sudo apt-get update` then `sudo apt-get install ros-indigo-ar-track-alvar`
   - http://wiki.ros.org/ar_track_alvar
1. `rosrun ar_track_alvar createMarker` to generate markers
   - There will be a menu to help you set the marker size, id, etc
1. Follow tutorial for UV texture mapping in Blender. Note I wasn't able to get Blender to run in the Ubuntu VM because of graphics drivers.
   - https://artisticrender.com/how-to-add-a-texture-to-an-object-in-blender/
1. Export the mesh as a .dae file. You might have to set the axes to align with Gazebo defaults (Gazebo uses a right-hand coordinate system where +Z is up (vertical), +X is forward (into the screen), and +Y is to the left)

1. https://gist.github.com/RDaneelOlivav/990addc733fbeb8549c3979d5bca41b2
   - gist for ar tag stuff

# to do
- figure out roslaunch `.launch` vs `.world`
- What is a URDF? Xacro is newer. Supposed to be more reusable
- how to use ros with gazebo and add the tag?
- install aruco ros package
- setup worlds with tags, baxter, and humanoid