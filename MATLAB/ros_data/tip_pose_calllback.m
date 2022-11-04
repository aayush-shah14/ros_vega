function tip_pose_calllback(~,msg)
    global tip_poses_t f1 xdes ydes TimeStepCount timeStep
    tip_poses_t=[tip_poses_t,msg.Data];
    figure(f1);
    plot(timeStep:timeStep:timeStep*TimeStepCount,tip_poses_t(1,:))
    hold on
    plot(timeStep:timeStep:timeStep*TimeStepCount,tip_poses_t(2,:))
    plot(timeStep:timeStep:timeStep*TimeStepCount,ydes(1:TimeStepCount))
    plot(timeStep:timeStep:timeStep*TimeStepCount,xdes(1:TimeStepCount))
%     plot(tip_poses_t(1,:))
%     hold on
%     plot(tip_poses_t(2,:))
%     legend('x','y')
    legend('x','y','ydes','xdes','Location','southwest')
    hold off
end