global xdes ydes
L=0.75;
yc=L/2.5;
yr=L/5;
xc=L/4;
xr=L/5;
y1=yc+zeros(1,150);
x1=xc-xr+zeros(1,150);
y2=yc+yr*sin(2*(pi/180)*(0:0.5:1000));
x2=xc-xr*cos(2*(pi/180)*(0:0.5:1000));
xdes=[x1,x2];
ydes=[y1,y2];