th = 0:2*pi/300:2*pi;
x = 0;
y = 0;
r = 2
 for i = th
       xp = r*cos(i);
       yp = r*sin(i);
       plot(xp,yp, 'o')
       hold on
 end