q3_h10 =[0,1/sqrt(2),1/sqrt(2),0;0,1/sqrt(2),-1/sqrt(2), 0;1,0,0,76.2;0,0,0,1]
q3_he1 = [1,0,0,146.05;0,0,1,255.325;0,-1,0,0;0,0,0,1]
a3 = q3_h10*q3_he1
q3 = [pi/4,0,0,0,0,0]


q4_h10 = [0,1,0,0;-1,0,0,0;0,0,1,0;0,0,0,1]*[0,1,0,0;0,0,1,0;1,0,0,76.2;0,0,0,1]
q4_h21 = [1,0,0,146.05;0,1,0,0;0,0,1,0;0,0,0,1]
q4_h32 = [1/sqrt(2),-1/sqrt(2),0,-187.325/sqrt(2);1/sqrt(2), 1/sqrt(2), 0, 187.325/sqrt(2);0,0,1,0;0,0,0,1]
q4_h43 = [-1,0,0,0;0,0,1,34;0,1,0,0;0,0,0,1]
q4_h54 = [0,-1,0,0;1,0,0,0;0,0,1,0;0,0,0,1]*[-1,0,0,0;0,-1,0,0;0,0,1,34;0,0,0,1]

q4_h20 = q4_h10*q4_h21
q4_h30 = q4_h10*q4_h21*q4_h32
a4 = q4_h10*q4_h21*q4_h32*q4_h43*q4_h54

q4_1 = [-pi/2,0,0,0,0,0]
q4_3 = [-pi/2,0,pi/4,0,0,0]
q4 = [-pi/2,0,pi/4,0,pi/2,0]



addpath('../Core') % references ROS interface and arm controller files you'll need for every lab

%connect to ROS : 
setenv('ROS_MASTER_URI','http://192.168.0.21:11311')
setenv('ROS_IP','192.168.0.11')

% start ROS 
con = rosStart(true); % start ROS with gripper enabled

% add command
q = q3;
% q = q4;
        
con = add_command(con,q);
disp("Now the current state value is : ...");
disp(con.cur_state);
disp("Finish the command!");

% shut down ROS
rosEnd; 