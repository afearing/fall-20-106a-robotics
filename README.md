# fall-20-106a-robotics
Repo for yoga baxter project


Meeting with Isabella
Goal: use baxter to move a mannequin into a pose
First milestone is something knocking into a mannequin
Might need to use bullet simulator for contact physics. Maybe don’t use gazebo. Gazebo does support bullet.
Have multiple benchmarks so we can make the project harder.
First priority is getting path planning working because it’s the hardest. Don’t worry about pose detection of human pose. Could get fancy with camera translating your pose from a real-world camera.


Goal: group of robots slamming together. Centralized brain. Just like midterm problem. Have it work for random map, random targets, random starting points for robots
Can add more rules to make it harder.

Go look for libraries and ideas for each project to narrow it down. What do we want the robots to do? Wha

# Ideas for yoga baxter
Start with rigid elbows: straight arms. Try to correct T and I poses to Y pose. Mannequin is basically a robot, but we don’t have motors in the joints. So it’s kinematics but Baxter is the motors. 
Talk more on thursday or wednesday. Thursday 5-7. 
Goal is to finish/understand lab 5
Look into mannequin
Find the right pathing algorithm (moveit)
Copy and paste necessary code to new project (make Repo)
Meeting with Ritika:
Start with Aruco tags
Place things in front of Baxter
How do we put Aruco tags in the Gazebo sim?
https://www.youtube.com/watch?v=WDhIaVOUwsk&feature=youtu.be
https://answers.ros.org/question/240392/add-ar-tag-in-gazebo/
https://www.theconstructsim.com/create-spawn-humans-gazebo7/
https://github.com/robotology/human-gazebo
https://github.com/CMU-Perceptual-Computing-Lab/openpose


Split up into
Aruco tags with blocks
Human brought in
Baxter cameras

Baxter interact with human

```
rosrun tf tf_echo [reference_frame] [target_frame]
rosrun tf tf_echo base left_hand
rostopic echo /fiducial_transforms
```
