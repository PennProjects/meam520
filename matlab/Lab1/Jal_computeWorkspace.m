clear
clc

addpath('../Core') % references ROS interface and arm controller files you'll need for every lab


gripper_positions = [];

for th_1  = -1.4 :0.5 :1.4
    for th_2 = -1.2 :0.5 : 1.4
        for th_3 = -1.8 : 0.5: 1.7
            for th_4 = -1.9 : 0.5 : 1.7
                for th_5 = -2 : 0.5 : 1.5
                        
                        q = [th_1, th_2, th_3, th_4, th_5, 0];
                        [jointPositions, T0e] = calculateFK(q);
                        gripper_positions = [gripper_positions; T0e([13,14,15])];
                                         
                end
            end
        end
    end                
end


% disp(gripper)
% scatter3(gripper_positions(:,1), gripper_positions(:,2), gripper_positions(:,3))

% plot the workspace
x = gripper_positions(:,1)
y = gripper_positions(:,2)
z = gripper_positions(:,3)

b_x = boundary(y,z)
b_y = boundary(x,z)
b_z = boundary(x,y)


plot3(x,y,z, 'o', 'Color','#FF0000')

grid on
xlabel('Xo')
ylabel('Yo')
zlabel('Zo')
xlim([1.1*min(x),1.1*max(x)])
ylim([1.1*min(y),1.1*max(y)])
zlim([1.1*min(z),1.1*max(z)])

hold on
plot3(x, 1.1*max(y)*ones(size(y)),z, '-', 'Color', '#add8e6')
plot3(x(b_y), 1.1*max(y)*ones(size(x(b_y))), z(b_y), '-', 'Color', '#FF0000')
plot3(1.1*max(x)*ones(size(x)),y,z, '-', 'Color', '#add8e6')
plot3(1.1*max(x)*ones(size(y(b_x))), y(b_x), z(b_x), '-', 'Color', '#FF0000')
plot3(x, y, 1.1*min(z)*ones(size(z)), '-', 'Color', '#add8e6')
plot3(x(b_z), y(b_z),1.1*min(z)*ones(size(x(b_z))) , '-', 'Color', '#FF0000')


 






