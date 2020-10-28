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

end