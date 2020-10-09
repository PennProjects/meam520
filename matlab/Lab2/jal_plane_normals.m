
clc
clear

d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 34;                        % Distance between joint 4 and end effector

% Target 1
% T0e = [[   0.019,    0.969,    0.245,   47.046];[   0.917,   -0.115,    0.382,   73.269];[   0.398 ,   0.217,   -0.891,  100.547];[   0.,       0. ,      0.,       1.]];

% Target 2
T0e = [[  -0.993,   -0.,       0.119,  -96.936];[   0.,      -1.,      -0.,       0.   ];[   0.119,    0.,       0.993,  401.229];[   0. ,      0.  ,     0.  ,     1.   ]];

% Target 3
% T0e =[ [-0.3409003, -0.1074855,  0.9339346, 282.96];[0.7842780, -0.5802868,  0.2194888, -48.302];[0.5183581,  0.8072881,  0.2821184, 235.071 ]; [0,0,0,1]];


% Decomposing T0e
r11 = T0e(1,1);
r12 = T0e(1,2);
r13 = T0e(1,3);
r21 = T0e(2,1);
r22 = T0e(2,2);
r23 = T0e(2,3);
r31 = T0e(3,1);
r32 = T0e(3,2);
r33 = T0e(3,3);
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% wrist center positions
x_c = x - (d4+d5) * r13; 
y_c = y - (d4+d5) * r23;
z_c = z - (d4+d5) * r33;
o_c = [x_c; y_c; z_c] ;



p1 = [0 0 0];
p2 = [0 0 50];
p3 = transpose(o_c);

normal = cross(p1-p2,p1-p3);
normal = 1*normal/norm(normal);


e_desired = [x,y,z];

x = dot(normal, e_desired)


%method 2
pointA = p1;
pointB = p2;
pointC = p3;
normal = cross(pointA-pointB, pointA-pointC); %# Calculate plane normal
%# Transform points to x,y,z
x = [pointA(1) pointB(1) pointC(1)];  
y = [pointA(2) pointB(2) pointC(2)];
z = [pointA(3) pointB(3) pointC(3)];

%Find all coefficients of plane equation    
A = normal(1); B = normal(2); C = normal(3);
D = -dot(normal,pointA);
%Decide on a suitable showing range
xLim=[min(x)*1.2 max(x)*2];
zLim = [min(z)*1.2 max(z)*1.2];

[X,Z] = meshgrid(xLim,zLim);
Y = (A * X + C * Z + D)/ (-B);
reOrder = [1 2  4 3];
figure();patch(X(reOrder),Y(reOrder),Z(reOrder),'b');
grid on;
view([30 30]);
alpha(0.3);

hold on
plot3(p1(1),p1(2),p1(3), '.', 'MarkerSize',50,'color', 'b')
plot3(p2(1),p2(2),p2(3), '.', 'MarkerSize',50, 'color', 'b')
plot3(p3(1),p3(2),p3(3), '.', 'MarkerSize',50, 'color', 'b')
plot3(e_desired(1),e_desired(2),e_desired(3),'.', 'MarkerSize',50,'color', 'r')

xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');






