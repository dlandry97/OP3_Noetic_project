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

## demo GUI
To run the demo gui, After launching the demo, in a new sourced terminal run `roslaunch op3_gui_demo op3_demo.launch`
This will give you a GUI that you can run walking demos and other actions.

# Action editor
To create and test new actions for the OP3, you can run the action editior using the command `roslaunch op3_action_editor op3_action_editor.launch`
This will open up the action editor software where you can create new actions. This is where I created the Human-walking gaits.
