clc
clear


addpath('maps')
map = loadmap('map2.txt');

qStart = [0,-0.5,0,0,0,0];
qGoal = [0.1 ,0.1,pi/4,0,0,0];

map = loadmap('map1.txt');



qCurr = qStart;
isDone = false;
path = [];

while(~isDone)
[qNext, isDone] = potentialFieldStep_jal(qCurr, map, qGoal);

if isnan(qNext)
%     qCurr
%     qNext
    break
end

path = [path; qNext];

qCurr = qNext;


% pause(0.1);
end

path = [qStart;
        path;
        qGoal]
    
    
