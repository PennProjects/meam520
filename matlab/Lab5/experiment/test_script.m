% controlling variable: zeta
clc
clear
addpath('../')
addpath('../maps')
qStart = [0 0 0 0 0 0];
qGoal = [0 -pi/4 pi/4 0 0 0]; 

map = loadmap('map2.txt');

% default 
zeta_default = 0.1*ones(6,3);
eta_default = 1e6*ones(6,3);
rhoa_default = 50*ones(6,1);
rhob_default = 20*ones(6,1);
stepsize_default = 0.08*ones(1,6);

zeta_value = [0.001, 0.1, 1, 10, 100];
% zeta_value = zeta_default(1,1)
eta_value = [0.001, 0.1, 1, 10 100]; 
rhoa_value = [5, 10, 50, 100, 200]; 
rhob_value = [5, 10, 50, 100, 200]; 

stepsize_value = [0.1, 0.08, 0.06, 0.02, 0.01]; 
col = ['#FF7F00';'#FFFF00'; '#00FF00';'#0000FF';'#FF0000'];
legend_i = cell(5,1)
for i=1:numel(zeta_value)
    col(i)
    %zeta
%     [path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_value(i)*ones(6,3), eta_default, rhoa_default, rhob_default, stepsize_default);
%     fprintf("Zeta:%d Number of Steps:%d\n",zeta_value(i),size(path,1))
    %eta
%     [path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_value(i)*ones(6,3), rhoa_default, rhob_default, stepsize_default);
%     fprintf("Eta:%d Number of Steps:%d\n",eta_value(i),size(path,1))
%     
    %rhoa
    [path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_value(i)*ones(6,1), rhob_default, stepsize_default);
    fprintf("Rho_a:%d Number of Steps:%d\n",rhoa_value(i),size(path,1))
    
    %rhob
    %[path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_default, rhob_value(i)*ones(6,1), stepsize_default);
    %fprintf("Rho_b:%d Number of Steps:%d\n",rhob_value(i),size(path,1))
    
    %stepsize
%     [path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_default,  rhob_default, stepsize_value(i)*ones(1,6));
%     fprintf("Step size:%d Number of Steps:%d\n",stepsize_value(i),size(path,1))
    
%     lineplot(end+1) = plot3(path(:,1),path(:,2),path(:,3),path(:,4), path(:,5), path(:, 6), 'LineWidth',2);


    legend_i{i} = strcat('rhoa : ',num2str(rhoa_value(i)), '     Path length :', num2str(size(path,1) ));
    
    hold on
    p(i,:) = plot3(path(:,1), path(:,2), path(:,3),'o-', 'LineWidth', 3, 'color', col(i,:)) 
    plot3(qStart(1), qStart(2), qStart(3), '.', 'MarkerSize',50, 'color', '#0E4D92')
    text(qStart(1), qStart(2), qStart(3), 'Start', 'FontSize', 15)
    plot3(qGoal(1), qGoal(2), qGoal(3),'.', 'MarkerSize',50, 'color', '#B80F0A')
    text(qGoal(1), qGoal(2), qGoal(3), 'Goal', 'FontSize', 15)
    
    grid on
    title(['Lynx robot Potential Field Planning'],'FontSize', 20, 'FontWeight', 'bold')
    xlabel('Theta 1', 'FontSize', 20, 'FontWeight', 'bold')
    ylabel('Theta 2', 'FontSize', 20, 'FontWeight', 'bold')
    zlabel('Theta 3', 'FontSize', 20, 'FontWeight', 'bold')
    view(3);
    
    hold off
%     legend(num2str(rhoa_value(i)))
%     legend(p)
end
legend(p, legend_i, 'FontSize',14)


% legend(lineplot,{'0.001','0.1','10','1000'})
% view(a,b)
% axis([-200 400,-200,400,-200,500])
% title('Trajectory')

% Plot view configuration

