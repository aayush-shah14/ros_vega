function force_calllback(~,msg)
    global force_t f3 des_pose_pub TimeStepCount xdes ydes pose_msg timeStep
    force_t=[force_t,msg.Data];
    TimeStepCount=TimeStepCount+1;
    figure(f3);
    plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(1,:))
    hold on
    plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(2,:))
    plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(3,:))
    plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(4,:))
    plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(5,:))
%     plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(6,:))
%     plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(7,:))
%     plot(timeStep:timeStep:timeStep*TimeStepCount,force_t(8,:))
    legend('tau1','tau2','tau3','tau4','tau5')
    hold off
    if(TimeStepCount<size(xdes,2))
        pose_msg.Data=single([xdes(TimeStepCount) ydes(TimeStepCount)]);
    else
        pose_msg.Data=single([xdes(size(xdes,2)) ydes(size(xdes,2))]);
    end
%     if(TimeStepCount<size(Q_des,2))
%         pose_msg.Data=single([Q_des(TimeStepCount) -Q_des(TimeStepCount) Q_des(TimeStepCount) -Q_des(TimeStepCount) Q_des(TimeStepCount)]);
%     else
%         pose_msg.Data=single([Q_des(800) -Q_des(800) Q_des(800) -Q_des(800) Q_des(800)]);
%     end
    send(des_pose_pub,pose_msg)
end