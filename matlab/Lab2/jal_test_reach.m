


% q = [0,0,0,pi/2,pi,2,0];
q = [0, 0.3456, -1.8, 0,0,0];

L = [76.2,146.05,187.325,34,34]; 

T01 = DHParam(0, -pi/2, L(1), q(5));
T12 = DHParam(L(2), 0, 0, q(2)-pi/2);
T23 = DHParam(L(3), 0, 0, q(3)+pi/2);
T34 = DHParam(0, -pi/2, 0,q(4)-pi/2);
T4e = DHParam(0, 0, L(4)+L(5), q(5));

t3e = T34*T4e;
t0e = T01 * T12 * T23 * T34 * T4e;


p1 = [0 0 0];

p2 = [0 0 1];

p3 = wrist_positiontest.calc_wrist_pos(L, q);




function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
hom_trans_matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];
end