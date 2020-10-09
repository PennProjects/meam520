
d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 34;                        % Distance between joint 4 and end effector

% Target 1
T0e = [[   0.019,    0.969,    0.245,   47.046];[   0.917,   -0.115,    0.382,   73.269];[   0.398 ,   0.217,   -0.891,  100.547];[   0.,       0. ,      0.,       1.]];

% Target 2
% T0e = [[  -0.993,   -0.,       0.119,  -96.936];[   0.,      -1.,      -0.,       0.   ];[   0.119,    0.,       0.993,  401.229];[   0. ,      0.  ,     0.  ,     1.   ]];

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



p1 = [0 0 100];
p2 = [0 0 50];
p3 = transpose(o_c);

normal = cross(p1-p2,p1-p3);
normal = 10*normal/norm(normal);
normal = transpose(normal);
normal = [normal, transpose(p1)];


patch(p1,p2,p3);
% % fill3(p1,p2,p3, 'r')
% % plot3(p1,p2,p3, '.')
% figure('color','w')
% h=patch('Faces',1:3,'Vertices',[p1;p2;p3]);
% set(h,'FaceColor','r','EdgeColor','k','LineWidth',2,'FaceAlpha',0.5)
% axis equal vis3d
% % view([30 30])
% 
% xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold')
% ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold')
% zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold')

% grid on
% alpha(0.3)
% hold on
% 
% plot3(normal(1, : ),normal(2, : ), normal(3, : ), 'Color','#000000', 'LineWidth', 5 )

xlim([-10,50])
ylim([-10,50])
zlim([-10,170])



