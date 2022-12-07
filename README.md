# OP3_walking_testbed
This is a guide to update the Robotis OP3 robot to ROS Noetic.

# Repository import
Create your workspace, `mkdir src`, and `cd src/`

Place the `OP3_Noetic.repos` file into `src/`

use command `vcs import < OP3_Noetic.repos`

This will import all of the required repositories to run the robot in ROS Noetic.

From here you can `cd` back into your workspace and catkin_make to build the packages.

# Running the demo
To run the demo on the robot
First soruce your workspace using `source devel/setup.bash`

then run the command `roslaunch op3_demo demo.launch`

## 
