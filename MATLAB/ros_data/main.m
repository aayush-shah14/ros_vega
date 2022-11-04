%% initializing ROS and variables
global tip_poses_t xt yt n timeStep nColsNodes deltaL original_pose f1 f2 f3 slopes_t force_t step_t des_pose_pub TimeStepCount pose_msg
timeStep=0.5;
force_t=[];
tip_poses_t=[];
step_t=[];
xt=[];
yt=[];
slopes_t=[];
TimeStepCount=0;
rosshutdown
rosinit
tip_pose_sub=rossubscriber("/actual_system/state","std_msgs/Float32MultiArray",@tip_pose_calllback,"DataFormat","struct");
% slope_pose_sub=rossubscriber("/actual_system/slope","std_msgs/Float32MultiArray",@slope_calllback,"DataFormat","struct");
tip_poseV_sub=rossubscriber("/steps_compiled","std_msgs/Float32MultiArray",@step_calllback,"DataFormat","struct");
state_sub=rossubscriber("/actual_system/complete_state","std_msgs/Float32MultiArray",@state_callback,"DataFormat","struct");
force_sub=rossubscriber("/actual_system/force","std_msgs/Float32MultiArray",@force_calllback,"DataFormat","struct");
des_pose_pub=rospublisher("/des_pose","std_msgs/Float32MultiArray","DataFormat","struct");
f1=figure;
f2=figure;
f3=figure;
% f4=figure;
pose_msg=rosmessage(des_pose_pub);
% pose_msg.Data=[xdes(1),ydes(1)];
% f4=figure('X step responses');
% f5=figure('Y step responses');
% f6=figure('slope step responses');

%% read configuration to know beam dimensions
f=csvread('~/catkin_ws/src/Soft_robo_sim/MATLAB/analysingVegaData/config_finer.csv');
n=f(1);
nColsNodes=f(2);
meshSize=nColsNodes-1;
BeamWidth=f(3);
deltaL=BeamWidth/meshSize;
original_pose=zeros(n,2);
nRows=n/nColsNodes;
nColumns=zeros(nRows,1);
nColumns(1:end)=meshSize;
count=1;
for i=0:(nRows-1)
    for j=0:nColumns(i+1,1)
        original_pose(count,:)=[-1*nColumns(i+1,1)*0.5*deltaL+j*deltaL,-deltaL*i];
        count=count+1;
    end
end