function [jointPositions,T0i] = calculateFK(q)
% CALCULATEFK This function calculates the forward kinematics for the
%   Lynx manipulator arm. 
%
% AUTHOR:
%   Dr. Cynthia Sung (crsung@seas.upenn.edu)
%   Modified by Gedaliah Knizhnik (knizhnik@seas.upenn.edu) 8/28/19
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0i            - a 4 x 4 x 6 list of homogeneous transformation matrix, 
%                    representing each joint frame expressed in the 
%                    base (0) frame. :,:,i is T0i
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

% Lynx ADL5 constants in mm
d1 = 76.2;                      % Distance between joint 0 and joint 1
a2 = 146.05;                    % Distance between joint 1 and joint 2
a3 = 187.325;                   % Distance between joint 2 and joint 3
d4 = 34;                        % Distance between joint 3 and joint 4
d5 = 68;                        % Distance between joint 3 and joint 5
lg = 0;                         % Distance between joint 5 and end effector (gripper length)

%Frame 1 w.r.t Frame 0
T1 = [cos(q(1)) -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)  0;
      sin(q(1))  cos(q(1))*cos(-pi/2) -cos(q(1))*sin(-pi/2)  0;
              0            sin(-pi/2)            cos(-pi/2) d1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
T2 = [cos(q(2)-(pi/2)) -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
      sin(q(2)-(pi/2))  cos(q(2)-(pi/2))  0   a2*sin(q(2)-(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
      sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
      sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
              0                          sin(-pi/2)                    cos(-pi/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
T5 = [cos(q(5)) -sin(q(5))  0        0;
      sin(q(5))  cos(q(5))  0        0;
              0          0  1       d5;
              0          0  0        1];
          
%Frame 5 w.r.t Frame 4 
T6 = [ 1  0  0   0;
       0  1  0   0;
       0  0  1  lg;
       0  0  0   1];

%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T1*[0;0;0;1])';

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T1*T2*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T1*T2*T3*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T1*T2*T3*T4*[0;0;d4;1])';

%Position of Gripper (Base of the Gripper)
X(6,:) = (T1*T2*T3*T4*T5*[0;0;0;1])';

%Outputs the 6x3 of the locations of each joint in the Base Frame
jointPositions = X(:,1:3);

T0i = zeros(4,4,6);
T0i(:,:,1) = eye(4);
T0i(:,:,2) = T1;
T0i(:,:,3) = T1*T2;
T0i(:,:,4) = T1*T2*T3;
T0i(:,:,5) = T1*T2*T3*T4;
T0i(:,:,6) = T1*T2*T3*T4*T5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end