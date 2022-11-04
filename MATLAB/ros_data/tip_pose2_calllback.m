function tip_pose2_calllback(~,msg)
    global tip_poses_t f4 f5 f6 step_count v s0 s1 s2
    if(msg.Data(end)==-1)
        v=msg.Data;
        step_count=step_count+1;
    elseif(msg.Data(end)==0)
        s0=msg.Data;
        step_count=step_count+1;
    elseif(msg.Data(end)==1)
        s1=msg.Data;
        step_count=step_count+1;
    else
        s2=msg.Data;
        step_count=step_count+1;
    end
    if(step_count==4)       
        figure(f4);
        clf(f4);
        hold on
        plot(tip_poses_t(1,:))
        hold on
        plot(tip_poses_t(2,:))
        plot(tip_poses_t(3,:))
        legend('x','y','slope')
        hold off
    end
end