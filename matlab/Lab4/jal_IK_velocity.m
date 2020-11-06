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
for i = 1:joint
    %Using formuala for Revolute joints : Jv = Z_i-1 x (O_n - O_i-1)
    J_v(:,i) = cross(z_0_i(i,:),(jointPositions(joint,:)-jointPositions(i,:)))';
    
    %Using formuala for Revolute joints : Jw = Z_i-1 x (O_6 - O_i-1)
    J_w(:,i) = (z_0_i(i,:))';
       
end



J = [J_v; J_w];
xi = [v; omega];

%Check for NaNs in xi

[no_nan_row, no_nan_col] = find(~isnan(xi));

%Keep only non nan rows
J = J(no_nan_row, :);
xi = xi(no_nan_row, :);



%checking for singular rows
rank_J = rank(J);
if rank_J <5
    J = round(J,3);
    [J_uniq, unique_index, repeat_index] = unique(J, 'rows', 'stable');
    xi_uniq = xi(unique_index, :)
else
    J_uniq = J;
    xi_uniq = xi;
end


%Checking feasibility
rank_J = rank(J_uniq);
rank_J_2 = rank([J_uniq xi_uniq]);


% if rank_J == rank_J_2
    % solution exists
    dq = pinv(J_uniq)* xi_uniq;
    
    % minimize least squares error 
% end



end