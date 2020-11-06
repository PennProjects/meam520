
clc
clear


lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];


q = [-1.3 0 0 -pi/2 0 0];
% q = [ 0 0 -0.5 0 0 0];
dq = [0 0 0 0 0 0]';
joint = 3;




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
% Z-Y PLANE
% v = [0 r*cos(th(i)) r*sin(th(i))]';

%X-Y PLANE
v = [r*cos(th(i)) r*sin(th(i)) 0]';
omega = [0 0 0]';

dq = jal_IK_velocity(q,v,omega,joint)




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
pause(0.0001)
grid on
drawnow 


i = i+1; 
q = q+dq
end 


