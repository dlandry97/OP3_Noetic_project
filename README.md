# OP3_walking_testbed
This is a guide to update the Robotis OP3 robot to ROS Noetic.

# Clonezilla (NU)
Import the clonezilla image from the provided USB.

First power down the robot and insert the USB stick wit hthe clonezilla image. Use one of the provided USB B slots for this. Also have a keyboard on hand and connected to the robot.

Power on the robot and press F10 until the boot menu appears. Select the USB drive that you inserted.

Clonezilla should start.

Follow the default setup and select image-disk. 

Continue following the promps until it asks for the image you want to clone. Select one of the images with Noetic in the name.

Follow the rest of the prompts. The cloneing process should take around 10-20 minutes. The robot should reboot or shut down. 

Safely store the USB.

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

More in depth documentation on this can be found at `https://emanual.robotis.com/docs/en/platform/op3/tutorials/#how-to-execute-gui-program`

# Action editor
To create and test new actions for the OP3, you can run the action editior using the command `roslaunch op3_action_editor op3_action_editor.launch`

Note: This is meant to be launched alone without any ROS nodes running.

This will open up the action editor software where you can create new actions. This is where I created the Human-walking gaits.

More information on the action editor can be found at `https://emanual.robotis.com/docs/en/platform/op3/tutorials/#how-to-execute-gui-program`
