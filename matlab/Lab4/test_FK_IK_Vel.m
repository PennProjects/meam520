
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


q = [0 0 0 0 0 0];
dq = [0 0 0 0 0 0];
joint = 6;
i = 0;
for i = 1:300
    
q = q+dq;
[jointPositions,T0e] = calculateFK(q);

v = [0 0.1 0]';
omega = [0 0 0]';

dq = jal_IK_velocity(q,v,omega,joint);




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
zlim([-10, 300])
% pause(0.01)
grid on
drawnow

end 


