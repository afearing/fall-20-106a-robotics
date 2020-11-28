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
1. Install `ar_track_alvar`. Use use `sudo apt-get update` then `sudo apt-get install ros-indigo-ar-track-alvar`
   - http://wiki.ros.org/ar_track_alvar
1. `rosrun ar_track_alvar createMarker` to generate markers
   - There will be a menu to help you set the marker size, id, etc
1. Follow tutorial for UV texture mapping in Blender. Note I wasn't able to get Blender to run in the Ubuntu VM because of graphics drivers.
   - https://artisticrender.com/how-to-add-a-texture-to-an-object-in-blender/
1. Export the mesh as a .dae file. You might have to set the axes to align with Gazebo defaults (Gazebo uses a right-hand coordinate system where +Z is up (vertical), +X is forward (into the screen), and +Y is to the left)
1. Follow this guide for how to start Gazebo with a custom world file. IDK how to do gazebo_ros though
   - http://gazebosim.org/tutorials/?tut=import_mesh


- `roslaunch turtlebot_gazebo turtlebot_world.launchworld_file:=$./worlds/room.world` this doesn't seem to work

# to do
- figure out roslaunch `.launch` vs `.world`
- What is a URDF? Xacro is newer. Supposed to be more reusable
- how to use ros with gazebo and add the tag?
- install aruco ros package
- setup worlds with tags, baxter, and humanoid