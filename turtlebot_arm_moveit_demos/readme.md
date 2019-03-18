# Introduction
## New head_tracker demo for the PhantomX Pincher
The new demo will let the arm to react when there is face detected by the kinect


## Setup and how to run it 
In the turtlebot_arm_bringup launch file, I have added new launch file

  * arm_moveit.launch --> For bringup the arm and moveit with kinect
  * arm_moveit_without_cam.launch --> For bringup the arm and moveit
  * usbcam_face.launch --> For launch usb camera and face tracker

## Necessary packages for the demonstration
 # rbx1 from pirobot 
    git clone https://github.com/IOJVision/rbx1.git
 # freenect launcher 
    build from source: https://github.com/ros-drivers/freenect_stack.git
    or by sudo apt-get install ros-indigo-freenect-camera ros-indigo-freenect-launch 


## Here is the list of command to run the head tracker program and bringup arm, rviz, kinect

## =======For the head tracker and move the arm=======

  # For bringup the arm and moveit
    roslaunch turtlebot_arm_bringup arm_moveit_withoutcam.launch

  # For bringup the arm and moveit with kinect
    roslaunch turtlebot_arm_bringup arm_moveit.launch

  # For launch usb camera and face tracker
    roslaunch turtlebot_arm_bringup usbcam_face.launch

  # For running the head tracker demo 
    rosrun turtlebot_arm_moveit_demos head_tracker.py

  # For running the face tracker
    roslaunch rbx1_vision face_tracker2.launch
