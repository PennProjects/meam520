% controlling variable: zeta

addpath('../')
q_start = [0 0 0 0 0 0];
q_goal = [0 -pi/4 -pi/4 0 0 0]; 

% default 
zeta_default = ones(6,3);
eta_default = 10*ones(6,3);
rhoa_default = 5*ones(6,1);
rhob_default = 5*ones(6,1);
stepsize_default = 0.01*ones(1,6);

zeta_value = [0.001, 0.1, 1, 10, 100];
eta_value = [0.001, 0.1, 1, 10 100]; 
rhoa_value = [5, 10, 50, 100, 200]; 
rhob_value = [5, 10, 50, 100, 200]; 

stepsize_value = []; 

lineplot = [];
for i=1:numel(value)
    %zeta
    [path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_value(i)*ones(6,3), eta_default, rhoa_default, rhob_default, stepsize_default)
    fprintf("Zeta:%d Number of Steps:%d\n",zeta_value(i),size(path,1))
    %eta
    %[path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_value(i)*ones(6,3), rhoa_default, rhob_default, stepsize_default);
    % fprintf("Eta:%d Number of Steps:%d\n",eta_value(i),size(path,1))
    
    %rhoa
    %[path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_value(i)*ones(6,1), rhob_default, stepsize_default);
    % fprintf("Rho_a:%d Number of Steps:%d\n",rhoa_value(i),size(path,1))
    
    %rhob
    %[path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_default, rhob_value(i)*ones(6,1), stepsize_default);
    %fprintf("Rho_b:%d Number of Steps:%d\n",rhob_value(i),size(path,1))
    
    %stepsize
    %[path] = experiment_potentialFieldPath(map, qStart, qGoal, zeta_default, eta_default, rhoa_default,  rhob_default, stepsize_value(i)*ones(1,6));
    %fprintf("Step size:%d Number of Steps:%d\n",stepsize_value(i),size(path,1))
    
    lineplot(end+1) = plot3(path(:,1),path(:,2),path(:,3),path(:,4), path(:,5), path(:, 6), 'LineWidth',2);

end

legend(lineplot,{'0.001','0.1','10','1000'})
view(a,b)
axis([-200 400,-200,400,-200,500])
title('Trajectory')