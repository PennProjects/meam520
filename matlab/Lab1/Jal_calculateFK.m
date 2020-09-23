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
    L5 = 34;      % distance between joint 4 and center of gripper

%% Your code here
    
%function to calculate the tranformation matrix for given set of DH parameters.
    function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
     
%    INPUT:
%       a        - link length, distance along x_i from the intersection of the x_1
%                  and z_i-1 axes to o_i.
%       alpha    - link twist, the angle between z_i-1 and z_i measured about
%                  x_i.
%       d        - link offset, distance along z_i-1 from o_i-1 to the intersection
%                  of the x_i and z_i-1 axes. d_i is variable if joint i is prismatic.
%       theta    - joint angle, the angle between x_i-1 and x_i measured about z_i-1. Theta_i
%                  is variable if joint i is revolute.
%
%   OUTPUT:
%       hom_trans_matrix   - 4 x 4 matrix, a homogenous transformation
%                            matrix that provies the ridig body tranformation, 
%                            including rotation and translation from
%                            frame_i to frame_i-1.


%    Calculates DHParams when given four parameters
        hom_trans_matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];
    end

    
%   Calculating joint angle of frames based on DH convention.
    theta_1 = q(1)+pi;
    theta_2 = q(2)+pi/2;
    theta_3 = q(3)+pi/2;
    theta_4 = q(4)+pi/2;
    theta_5 = q(5)+pi;

%   Calculation of tranformation matrix Tij - From frame_j to frame_i
    T01 = DHParam(0, pi/2, L1, theta_1);
    T12 = DHParam(L2, 0, 0, theta_2);
    T23 = DHParam(L3, 0, 0, theta_3);
    T34 = DHParam(0, pi/2, 0, theta_4);
    T4e = DHParam(0, 0, L4+L5, theta_5);

%   Calculation of pposition of Origins Oi of frame_i in global(base) coordinates.
    O0 = [0,0,0]
    O1 = T01
    O2 = T01 * T12
    O3 = T01 * T12 * T23
    O4 = T01 * T12 * T23 * T34
    Oe = T01 * T12 * T23 * T34 * T4e
    
%   Calculation of joint positions of each joint Ji
    J1 = O0;
    J2 = O1([13 14 15]); 
    J3 = O2([13 14 15]);
    J4 = O3([13 14 15]);
    J5_t = O4 * [0; 0 ;L4; 1];
    J5 = transpose(J5_t([1 2 3]));   % Joint 5 is off the origin O4 by L4 along the z-axis 
    Je = Oe([13 14 15]);

% Output of function, Transformation matrix from e to 0 and 6 joint positions
    T0e = T01 * T12 * T23 * T34 * T4e;
    jointPositions = [ J1; J2; J3; J4; J5; Je];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end