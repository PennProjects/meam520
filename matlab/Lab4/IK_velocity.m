  function [dq] = IK_velocity(q, v, omega, joint)
% function [dq] = IK_velocity(q, v, omega, joint)
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q     - 1 x 6 vector corresponding to the robot's current configuration 
%   v     - The desired linear velocity in the world frame. If any element
%           is Nan, then that velocity can be anything
%   omega - The desired angular velocity in the world frame.
%           If any element is Nan, then that velocity can be anything
%   joint - an integer in [0,6] corresponding to which joint you are
%           tracking
%
% OUTPUT:
%   dq - 1 x 6 vector coresponding to the joint velocities. If v and omega
%        are infeasible, then dq should minimize the least squares error.
%        
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
%Using formuala for Revolute joints : Jv = Z_i-1 x (O_6 - O_i-1)
J_v(:,1) = cross(z_0_i(1,:),(jointPositions(6,:)-jointPositions(1,:)))';
J_v(:,2) = cross(z_0_i(2,:),(jointPositions(6,:)-jointPositions(2,:)))';
J_v(:,3) = cross(z_0_i(3,:),(jointPositions(6,:)-jointPositions(3,:)))';
J_v(:,4) = cross(z_0_i(4,:),(jointPositions(6,:)-jointPositions(4,:)))';
J_v(:,5) = cross(z_0_i(5,:),(jointPositions(6,:)-jointPositions(5,:)))';
J_v(:,6) = cross(z_0_i(6,:),(jointPositions(6,:)-jointPositions(6,:)))';


%Using formuala for Revolute joints : Jw = Z_i-1 x (O_6 - O_i-1)
J_w(:,1) = (z_0_i(1,:))';
J_w(:,2) = (z_0_i(2,:))';
J_w(:,3) = (z_0_i(3,:))';
J_w(:,4) = (z_0_i(4,:))';
J_w(:,5) = (z_0_i(5,:))';
J_w(:,6) = (z_0_i(6,:))';

%Compiling Linear velocity for only interested joint
J_v_ = J_v(:, 1:joint);
J_w_ = J_w(:, 1:joint);

J = [J_v_; J_w_];
xi = [v; omega];

rank_J = rank(J);

rank_J_2 = rank([J xi]);

% J singularity
% xi is infeasible
% NaNs

if rank_J == rank_J_2
    % solution exists
    dq = J \ xi;
    % minimize least squares error 
end



end