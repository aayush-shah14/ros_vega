global xdes ydes
L=0.75;
y1=L/2+zeros(1,200);
y2=L/2:-L/190:L/4;
x1=L/8+zeros(1,200+size(y2,2));
x2=L/8:L/200:L/2;
x3=L/2+zeros(1,870);
y3=L/4+zeros(1,size(x2,2)+size(x3,2));
xdes=[x1,x2,x3];
ydes=[y1,y2,y3];