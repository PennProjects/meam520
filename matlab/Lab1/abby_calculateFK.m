function [jointPositions,T0e] = calculateFK(q)
% CALCULATEFK - 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
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
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lynx Dimensions in mm
L1 = 76.2;    % distance between joint 0 and joint 1
L2 = 146.05;  % distance between joint 1 and joint 2
L3 = 187.325; % distance between joint 2 and joint 3
L4 = 34;      % distance between joint 3 and joint 4
L5 = 68;      % distance between joint 4 and center of gripper

%% Your code here
    function [matrix] = DHParam(a, alpha, d, theta)
        % Outputs DHParams when given four parameters
        matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                  sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                  0, sin(alpha), cos(alpha), d;
                  0, 0, 0, 1];
    end

theta_1 = q(1)+pi;
theta_2 = q(2)+pi/2;
theta_3 = q(3)+pi/2;
theta_4 = q(4)+pi/2;
theta_5 = q(5)+pi;

T10 = DHParam(0, pi/2, L1, theta_1);
T21 = DHParam(L2, 0, 0, theta_2);
T32 = DHParam(L3, 0, 0, theta_3);
T43 = DHParam(0, pi/2, 0, theta_4);
Te4 = DHParam(0, 0, L4+L5, theta_5);

% origins of the 6
%O0_init = [0 0 0 1];
%O1_init = [0 0 L1 1];
%O2_init = [0 0 L1+L2 1];
%O3_init = [L3 0 L1+L2 1];
%O4_init = [L3 0 L1+L2 1];
%Oe_init = [L3+L4+L5 0 L1+L2 1];

% joint pos of 6
O1 = T10;
O2 = T10 * T21 ;
O3 = T10 * T21 * T32;
O4 = T10 * T21 * T32 * T43;
Oe = T10 * T21 * T32 * T43 * Te4;

T0e = T10 * T21 * T32 * T43 * Te4;
jointPositions = [ 0 0 0; O1([13 14 15]); O2([13 14 15]); O3([13 14 15]); O4([13 14 15]); Oe([13 14 15])];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end