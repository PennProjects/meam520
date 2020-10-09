
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
% T0e = [[  -0.993,   -0.,       0.119,  -96.936];[   0.,      -1.,      -0.,       0.   ];[   0.119,    0.,       0.993,  401.229];[   0. ,      0.  ,     0.  ,     1.   ]];

% Target 3
T0e =[ [-0.3409003, -0.1074855,  0.9339346, 282.96];[0.7842780, -0.5802868,  0.2194888, -48.302];[0.5183581,  0.8072881,  0.2821184, 235.071 ]; [0,0,0,1]];

% Target 4lab2  
% T0e =[[  0.5054096, -0.8370580, -0.2095115, -45];[-0.0305796,  0.2252773, -0.9738147,-300];[0.8623375,  0.4985821,  0.0882604, 63 ];[0,0,0,1]];



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
o_c = [x_c y_c z_c] ;



p1 = [0 0 0];
p2 = [0 0 50];
p3 = o_c;


normal_vec = cross(p1-p2,p1-p3);
normal_vec = 1*normal_vec/norm(normal_vec)

normal_line = [o_c; (o_c+normal_vec*10)];


e_desired = [x,y,z];
e_norm = e_desired/norm(e_desired);
e_from_wrist = e_desired-o_c;
e_proj_on_normal = dot(e_from_wrist,normal_vec)*normal_vec;
e_possible_from_wrist =  e_from_wrist-e_proj_on_normal;
e_possible = e_possible_from_wrist+o_c
e_pos_norm = e_possible/norm(e_possible);

not_in_plane = round(dot(normal_vec, e_norm),3);
is_in_plane = round(dot(normal_vec, e_pos_norm),3);



e_desired_zaxs = [e_desired; (e_desired+[r13,r23,r33]*10)];
e_desired_yaxs = [e_desired; (e_desired+[r12,r22,r32]*10)];
e_desired_xaxs = [e_desired; (e_desired+[r11,r21,r31]*10)];

e_desired_zaxis_unitvector = [r13,r23,r33];
e_desired_zaxis_unitvector_prj_on_normal = dot(e_desired_zaxis_unitvector,normal_vec)*normal_vec;
e_possible_zaxis_vector = e_desired_zaxis_unitvector-e_desired_zaxis_unitvector_prj_on_normal;
e_possible_zaxis_norm = e_possible_zaxis_vector/norm(e_possible_zaxis_vector);


e_possible_zaxs = [e_possible;(e_possible+e_possible_zaxis_norm*20)]; 



[X,Y,Z,reOrder] = makeMesh(p1,p2,p3);
figure();
patch(X(reOrder),Y(reOrder),Z(reOrder),'b');
alpha(0.3);



hold on
plot3(p1(1),p1(2),p1(3), '.', 'MarkerSize',50,'color', 'b')
plot3(p2(1),p2(2),p2(3), '.', 'MarkerSize',50, 'color', 'b')
plot3(p3(1),p3(2),p3(3), '.', 'MarkerSize',50, 'color', 'b')
plot3(normal_line(:,1), normal_line(:,2), normal_line(:,3), 'LineWidth', 5,'color', '#000000')


plot3(e_desired(1),e_desired(2),e_desired(3),'.', 'MarkerSize',50,'color', 'r')
plot3(e_desired_zaxs(:,1), e_desired_zaxs(:,2), e_desired_zaxs(:,3), 'LineWidth', 3,'color', 'b')
plot3(e_desired_yaxs(:,1), e_desired_yaxs(:,2), e_desired_yaxs(:,3), 'LineWidth', 3,'color', 'g')
plot3(e_desired_xaxs(:,1), e_desired_xaxs(:,2), e_desired_xaxs(:,3), 'LineWidth', 3,'color', 'r')


plot3(e_possible(1),e_possible(2), e_possible(3), '.', 'MarkerSize',50,'color', 'g')
plot3(e_possible_zaxs(:,1), e_possible_zaxs(:,2), e_possible_zaxs(:,3), 'LineWidth', 3,'color', 'b')

%fin
grid on;
view([30 30]);
xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold');
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold');
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold');



function [X,Y,Z, reOrder] = makeMesh(p1,p2,p3)

%plot
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
end




