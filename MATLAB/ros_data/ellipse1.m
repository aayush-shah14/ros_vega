global xdes ydes
L=0.75;
yc=L/4;
yr=L/8;
xc=0;
xr=L/6;
y1=yc+zeros(1,150);
x1=xc-xr+zeros(1,150);
y2=yc+yr*sin(2*(pi/180)*(0:0.5:1000)); %slow one plot 6 was at 250 frequency
x2=xc-xr*cos(2*(pi/180)*(0:0.5:1000));
xdes=[x1,x2];
ydes=[y1,y2];