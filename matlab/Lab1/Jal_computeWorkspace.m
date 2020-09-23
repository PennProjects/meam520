clear
clc

% To compute the workspace
% We do this by computing all the poistions the gripper(Je/Origin of frame_5) can reach.
% This is done by looping through the various positions of all the joints. 
% The step size for the angles can be varied to get better resolution.
% Needs calculateFK.m to be in the same folder as this file

gripper_positions = [];

 for th_1  = -1.4 :0.3 :1.4  
    for th_2 = -1.2 :0.5 : 1.4
        for th_3 = -1.8 : 0.5: 1.7
            for th_4 = -1.9 : 0.5 : 1.7
                for th_5 = -2 : 0.5 : 1.5
                        
                        q = [th_1, th_2, th_3, th_4, th_5, 0];
                        [~, T0e] = calculateFK(q);
                        gripper_positions = [gripper_positions; T0e([13,14,15])];
                                         
                end
            end
        end
    end                
end


% Plotting the workspace

% There are the possible x,y,z positions of the gripper
x = gripper_positions(:,1);
y = gripper_positions(:,2);
z = gripper_positions(:,3);

% Calculating the boundary of the projection on the axes and 3D surface at the boundary 
b_x = boundary(y,z);
b_y = boundary(x,z);
b_z = boundary(x,y);
b_3d = boundary(gripper_positions);

% Axes lines for plotting 
x_axs = [1.1*min(x),1.1*max(x)];
y_axs = [1.1*min(y),1.1*max(y)];
z_axs = [1.1*min(z),1.1*max(z)];

% Calculating joint poistions at Zero configuration to show dummy robot in 3D
q = [0,0,0,0,0,0];
[jointPositions, T0e] = calculateFK(q);

% 3D Plot of all points where the gripper can reach
plot3(x,y,z, 'o', 'Color','#E26400')
set(gca,'Color','k')

% Plot view configuration
grid on
xlabel('Xo', 'FontSize', 20, 'FontWeight', 'bold')
ylabel('Yo', 'FontSize', 20, 'FontWeight', 'bold')
zlabel('Zo', 'FontSize', 20, 'FontWeight', 'bold')
xlim([1.1*min(x),1.1*max(x)])
ylim([1.1*min(y),1.1*max(y)])
zlim([1.1*min(z),1.1*max(z)])

% Plotting the projections on the XY, YZ and ZX Axes
hold on
% plot3(x, 1.1*max(y)*ones(size(y)),z, ':', 'Color', '#add8e6','LineWidth', 1.5 )
% plot3(1.1*max(x)*ones(size(x)),y,z, ':', 'Color', '#add8e6','LineWidth', 1.5 )
% plot3(x, y, 1.1*min(z)*ones(size(z)), ':', 'Color', '#add8e6','LineWidth', 1.5 )

% Plotting boundary of the porjection
% plot3(x(b_y), 1.1*max(y)*ones(size(x(b_y))), z(b_y), '-', 'Color', '#FF0000')
% plot3(1.1*max(x)*ones(size(y(b_x))), y(b_x), z(b_x), '-', 'Color', '#FF0000')
% plot3(x(b_z), y(b_z),1.1*min(z)*ones(size(x(b_z))) , '-', 'Color', '#FF0000')

% Plotting surfacre /3D boundary
trisurf(b_3d,x,y,z, 'Facecolor', '#E26400', 'FaceAlpha', '0.3')

% plot dummy robot at zero confoguration
% plot3(jointPositions(:,1),jointPositions(:,2), jointPositions(:,3),'d-','Color','#000000', 'LineWidth', 5)

% plot Axes
% plot3(x_axs, zeros(size(x_axs)), zeros(size(x_axs)), 'Color','#A9A9A9', 'LineWidth', 3)
% plot3(zeros(size(y_axs)), y_axs, zeros(size(y_axs)), 'Color','#A9A9A9', 'LineWidth', 3)
% plot3(zeros(size(z_axs)), zeros(size(z_axs)), z_axs, 'Color','#A9A9A9', 'LineWidth', 3)


% JARVIS Color : #E26400
% set(gca,'Color','k')







 






