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


%Cleaned Matrix for symbollic calculation
T01 = [cos(q(1)) 0  -sin(q(1))  0;
      sin(q(1)) 0 cos(q(1))  0;
              0            -1            0 d1;
              0                     0                  0     1];
%Cleaned Matrix for symbollic calculation          
T12 = [sin(q(2)) cos(q(2))  0   a2*sin(q(2));
      -cos(q(2))  sin(q(2))  0   -a2*cos(q(2));
              0                        0  1                     0;
              0                        0  0                     1];
%Cleaned Matrix for symbollic calculation         
T23 = [-sin(q(3)) -cos(q(3))  0   -a3*sin(q(3));
      cos(q(3))  -sin(q(3))  0   a3*cos(q(3));
              0                        0  1                     0;
              0                        0  0                     1];
%Cleaned Matrix for symbollic calculation          
T34 = [sin(q(4)) 0   cos(q(4))   0;
      -cos(q(4))  0  sin(q(4))   0;
              0                          -1                    0   0;
              0                                   0                             0   1];
          %Cleaned Matrix for symbollic calculation
T4e = [cos(q(5)) -sin(q(5))  0        0;
      sin(q(5))  cos(q(5))  0        0;
              0          0  1       d5;
              0          0  0        1];
% Computing the Transformation matrix to Frame 0          
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T0e = T04*T4e;

%Calculating the Zaxis of frame i wrt to Frame 0
z_0_i = [[0 0 1];
          T01([9 10 11]);
          T02([9 10 11]);
          T03([9 10 11]);
          T04([9 10 11])
          T0e([9 10 11])];   
      
%Calculating the position of orgin of each joint

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T02*[0;0;0;1])';

%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T01*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T03*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T04*[0;0;d4;1])';

%Position of Gripper (Base of the Gripper)
X(6,:) = (T0e*[0;0;0;1])';


%Calculating the locations of each joint in the Base Frame
jointPositions = X(:,1:3);

%Calculating linear joint velocity 

for i = 1:joint
    %Using formuala for Revolute joints : Jv = Z_i-1 x (O_n - O_i-1)
    J_v(:,i) = cross(z_0_i(i,:),(jointPositions(joint,:)-jointPositions(i,:)))';
    
    %Using formuala for Revolute joints : Jw = Z_i-1 x (O_6 - O_i-1)
    J_w(:,i) = (z_0_i(i,:))';
       
end



%reducing dq for joint of interest
dq_ = dq(1:joint)';


%Compiling Linear velocity
v = (J_v*dq_);

omega = (J_w*dq_);
end