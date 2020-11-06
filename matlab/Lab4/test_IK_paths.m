
clc
clear
% q = [0 0 pi/2 0 0 0];
% dq= [0 0.1 0 0 0 0];
% 
% joint = 6;
% 
% [v, omega] = jal_FK_velocity(q,dq,joint)
% 
% 
% dq_ = jal_IK_velocity(q,v,omega,joint)

lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];


q = [0 0 0 0 0 0];
dq = [0 0 0 0 0 0]';
joint = 6;


a1 = animatedline('Color',[0 .7 .7]);

size = 300;
trajectory = [];

th = 0:2*pi/size:2*pi+0.1;
i = 1;
while (i < size & q>lowerLim  & q< upperLim)
   

[jointPositions,T0e] = calculateFK(q);

% % straight line
% v = [0 1 0]';
% omega = [0 0 0]';


% %circle
th = 0:2*pi/300:2*pi;
r = 1;
v = [0 r*cos(th(i)) r*sin(th(i))]'
omega = [0 0 0]'

dq = jal_IK_velocity(q,v,omega,joint);




% plot dummy robot at zero confoguration


trajectory = [trajectory;jointPositions(6,:)];


plot3(jointPositions(:,1),jointPositions(:,2), jointPositions(:,3),'d-','Color','#000000', 'LineWidth', 5)
hold on
plot3(trajectory(:,1),trajectory(:,2), trajectory(:,3),'d-','Color','r', 'LineWidth', 2)
hold off





 % Plot view configuration

title('Lynx robot workspace','FontSize', 30, 'FontWeight', 'bold')
xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold')
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold')
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold')  

xlim([-100, 500])
ylim([-100 500])
zlim([-100, 500])
% pause(0.1)
grid on
drawnow 


i = i+1; 
q = q+dq'
end 


