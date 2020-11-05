
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
dq = [0 0 0 0 0 0];
joint = 6;
i = 0;
while (i < 3 & q>lowerLim  & q< upperLim)
    
q = q+dq


[jointPositions,T0e] = calculateFK(q)

v = [0 0 0]';
omega = [0 0 0]';

dq = jal_IK_velocity(q,v,omega,joint)




% plot dummy robot at zero confoguration
plot3(jointPositions(:,1),jointPositions(:,2), jointPositions(:,3),'d-','Color','#000000', 'LineWidth', 5)
% hold on 

 % Plot view configuration

title('Lynx robot workspace','FontSize', 30, 'FontWeight', 'bold')
xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold')
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold')
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold')  

xlim([-10, 300])
ylim([-10 30])
zlim([-300, 500])
% pause(0.01)
grid on
drawnow limitrate
% hold on

i = i+1;    
end 

