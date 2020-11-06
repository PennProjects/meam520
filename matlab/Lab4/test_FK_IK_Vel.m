
clc
clear
q = [0    1  0         0         0         0];
dq= [0.45647 0 0 0 0 0];

% v = [0 0 NaN]';
% omega = [0 0 0]';
joint = 6;

[v, omega] = jal_FK_velocity(q,dq,joint)


dq_ = jal_IK_velocity(q,v,omega,joint)




