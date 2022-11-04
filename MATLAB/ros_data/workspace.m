clear
clc
%% initializing ROS and variables
global stept tipPoset
stept=[];
tipPoset=[];
rosshutdown
rosinit
tip_pose_eq_sub=rossubscriber("/state_eq","std_msgs/Float32MultiArray",@eq_pose,"DataFormat","struct");
step_eq_sub=rossubscriber("/step_eq","std_msgs/Float32MultiArray",@eq_step,"DataFormat","struct");