function [q isPos] = calculateIK(T0e)
% CALCULATEIK - Please rename this function using your group # in
%   both the function header and the file name. 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 68;                        % Distance between joint 4 and end effector

% Joint limits
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

%% Your code here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end