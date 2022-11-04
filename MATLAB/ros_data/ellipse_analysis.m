nOutput=4;
nSections=5;
step=step_t(:,320);
step_matrix=zeros(2,nSections);
for i=0:(2-1)
    step_matrix(i+1,:)=step((1+5*i):5*(i+1));
end

[u,sigma,v]=svd(step_matrix);

%%
% [m,n]=min(abs(tipPoset(1,:)))
f=csvread('config_finer.csv');
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
L=0.75;
pause(10)
for i=0:(nRows-1)
    for j=0:nColumns(i+1,1)
        original_pose(count,:)=[-1*nColumns(i+1,1)*0.5*deltaL+j*deltaL,-deltaL*i];
        count=count+1;
    end
end
hold on

%%
i=320;
x=xt(:,i)+original_pose(:,1);
y=yt(:,i)+original_pose(:,2);
plot(x(ceil(nColsNodes*0.5):nColsNodes:end),y(ceil(nColsNodes*0.5):nColsNodes:end),'k','LineWidth',3)
yc=-0.75+tip_poses_t(2,320);
xc=tip_poses_t(1,320);
norm=sqrt(0.4715^2+0.0189^2)*30;
yr=0.0189/norm;
xr=0.4715/norm;
y2=yr*sin(2*(pi/180)*(0:0.5:1000));
x2=xr*cos(2*(pi/180)*(0:0.5:1000));
jj=u*[x2;y2];
plot(jj(1,:)+xc,jj(1,:)+yc)
xlabel('x in m')
ylabel('y in m')
title('ellipsiod')