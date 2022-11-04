function state_callback(~,msg)
    global xt yt nColsNodes original_pose f2
    x=msg.Data(1:2:end);
    y=msg.Data(2:2:end);
    xt=[xt,x];
    yt=[yt,y];
    x=x+original_pose(:,1);
    y=y+original_pose(:,2);
    figure(f2);
    plot(x(1:nColsNodes:end),y(1:nColsNodes:end),'k','LineWidth',2) %line1
    hold on
    plot(x(nColsNodes:nColsNodes:end),y(nColsNodes:nColsNodes:end),'k','LineWidth',2) %line3 flip
    plot(x(ceil(nColsNodes*0.5):nColsNodes:end),y(ceil(nColsNodes*0.5):nColsNodes:end),'k--','LineWidth',1)
    plot(x((end-nColsNodes+1):end),y((end-nColsNodes+1):end),'k','LineWidth',2) %line2
    plot(x(1:nColsNodes),y(1:nColsNodes),'b','LineWidth',4) %line4 flip
    hold off
    xlim([-0.05+original_pose(end,2)*0.5,0.05-original_pose(end,2)*0.6])
    ylim([original_pose(end,2)*1.2,0.1])
end