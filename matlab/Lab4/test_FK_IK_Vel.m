
clc
clear
q = [0 0 0 0 0 0];
dq= [0 0.1 0 0 0 0];

joint = 6;

[v, omega] = jal_FK_velocity(q,dq,joint)