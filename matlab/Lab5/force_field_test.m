

clc
clear

qCurr = [5 5 5 5 5 5];
qGoal = [8 9 6 8 9 6]; 

rho_a  = [2 2 2 2 2 2];
zeta = [1 1 1 1 1 1];

distance_from_goal = abs(qCurr-qGoal)

f_att = [];



% if(distance_from_goal <= rho_a)
    x = distance_from_goal <= rho_a
    f_att(x) = -zeta(x).*(qCurr(x)-qGoal(x))
% else
    y = distance_from_goal > rho_a
    f_att(y)= -rho_a(y).*zeta(y).*((qCurr(y)-qGoal(y))/norm(qCurr(y)-qGoal(y)))
% end