function [v omega] = FK_velocity(q, dq, joint)
% function [v omega] = FK_velocity(q dq joint)
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q  - 1 x 6 vector corresponding to the robot's current configuration
%   dq - 1 x 6 vector corresponding to the robot's current joint velocities
%   joint - an integer in [0,6] corresponding to which joint you are
%           tracking
%
% OUTPUT:
%   v     - The resulting linear velocity in the world frame
%   omega - The resulting angular velocity in the world frame
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here
d1 = 76.2;                      % Distance between joint 0 and joint 1
a2 = 146.05;                    % Distance between joint 1 and joint 2
a3 = 187.325;                   % Distance between joint 2 and joint 3
d4 = 34;                        % Distance between joint 3 and joint 4
d5 = 68;                        % Distance between joint 3 and joint 5
lg = 0;                         % Distance between joint 5 and end effector (gripper length)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Computing Transformation Matices
%Frame 1 w.r.t Frame 0
T01 = [cos(q(1)) -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)  0;
      sin(q(1))  cos(q(1))*cos(-pi/2) -cos(q(1))*sin(-pi/2)  0;
              0            sin(-pi/2)            cos(-pi/2) d1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
T12 = [cos(q(2)-(pi/2)) -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
      sin(q(2)-(pi/2))  cos(q(2)-(pi/2))  0   a2*sin(q(2)-(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
T23 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
      sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
T34 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
      sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
              0                          sin(-pi/2)                    cos(-pi/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
T45 = [cos(q(5)) -sin(q(5))  0        0;
      sin(q(5))  cos(q(5))  0        0;
              0          0  1       d5;
              0          0  0        1];
          
%Frame 6 w.r.t Frame 5 
T56 = [ 1  0  0   0;
       0  1  0   0;
       0  0  1  lg;
       0  0  0   1];
   
   
% Computing the Transformation matrix to Frame 0          
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

%Calculating the Zaxis of frame i wrt to Frame 0
z_0_i = [[0 0 1];
          T01([9 10 11]);
          T02([9 10 11]);
          T03([9 10 11]);
          T04([9 10 11]);
          T05([9 10 11]);
          T06([9 10 11])];
      
%Calculating the position of orgin of each joint

%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T01*[0;0;0;1])';

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T02*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T03*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T04*[0;0;d4;1])';

%Position of Gripper (Base of the Gripper)
X(6,:) = (T05*[0;0;0;1])';


%Calculating the locations of each joint in the Base Frame
jointPositions = X(:,1:3);

%Calculating manipulator Jacobian upto joint of interest.
for i = 1:joint
    %Using formuala for Revolute joints : Jv = Z_i-1 x (O_n - O_i-1)
    J_v(:,i) = cross(z_0_i(i,:),(jointPositions(joint,:)-jointPositions(i,:)))';
    
    %Using formuala for Revolute joints : Jw = Z_i-1 x (O_n - O_i-1)
    J_w(:,i) = (z_0_i(i,:))';
       
end

%Constructing the manipulator Jacobian J and body velocity xi
J = [J_v; J_w];
%reducing dq for joint of interest
dq = dq(1:joint)';


%Computing Linear velocity
v = (J_v*dq);

%Computing Angular velocity
omega = (J_w*dq);


end