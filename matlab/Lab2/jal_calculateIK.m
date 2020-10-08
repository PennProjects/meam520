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
d5 = 34;                        % Distance between joint 4 and end effector

% Joint limits
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

%% Your code here
% Decomposing T0e
r11 = T0e(1,1);
r12 = T0e(1,2);
r13 = T0e(1,3);
r21 = T0e(2,1);
r22 = T0e(2,2);
r23 = T0e(2,3);
r31 = T0e(3,1);
r32 = T0e(3,2);
r33 = T0e(3,3);
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% wrist center positions
x_c = x - (d4+d5) * r13; 
y_c = y - (d4+d5) * r23;
z_c = z - (d4+d5) * r33;
o_c = [x_c; y_c; z_c] ;

% if not feasible
% not reachable workspace 
%  triangle inequality? 

% isPos=false
% q = [0 0 0 0 0]
% return 
% find pos of the center 
% from wrist center - check if any of the angles are within the limits 

% define qs

q1 = atan2(y_c,x_c);
if q1==pi
    q1=0;
end




c1 = cos(q1);
s1 = sin(q1);

gamma = acos((a2^2 + a3^2 - x_c^2 - y_c^2 - (z_c-d1)^2) / (2*a2*a3));


q3_a(1) = pi/2-gamma;
q3_a(2) = gamma-3*pi/2 ;


c3_a = cos(q3_a); 
s3_a = sin(q3_a);


% theta 2
q2_a(1) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(2) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(3) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(4) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));

q2_a(5) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(6) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(7) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(8) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));

c2_a = cos(q2_a);
s2_a = sin(q2_a);


% calculate theta 4 and theta 5
q = [q1, q2_a(1), q3_a(1), 0,0];
L = [d1,a2,a3,d4,d5];

T01 = DHParam(0, -pi/2, L(1), q(1));
T12 = DHParam(L(2), 0, 0, q(2)-pi/2);
T23 = DHParam(L(3), 0, 0, q(3)+pi/2);
% T34 = DHParam(0, -pi/2, 0,q(4)-pi/2);
% T4e = DHParam(0, 0, L(4)+L(5), q(5));

T03 = T01 * T12 * T23;
R03 = T03(1:3, 1:3);
R0e = T0e(1:3,1:3);
R3e = transpose(R03)*R0e;

q4_a(1) = atan2(R3e(2,3), R3e(1,3));
q5_a(1) = atan2(-R3e(3,1),-R3e(3,2)); %check why -ve sign is required


% using second angle

q = [q1, q2_a(2), q3_a(2), 0,0];
L = [d1,a2,a3,d4,d5];

T01 = DHParam(0, -pi/2, L(1), q(1));
T12 = DHParam(L(2), 0, 0, q(2)-pi/2);
T23 = DHParam(L(3), 0, 0, q(3)+pi/2);
% T34 = DHParam(0, -pi/2, 0,q(4)-pi/2);
% T4e = DHParam(0, 0, L(4)+L(5), q(5));

T03 = T01 * T12 * T23;
R03 = T03(1:3, 1:3);
R0e = T0e(1:3,1:3);
R3e = transpose(R03)*R0e;

q4_a(2) = atan2(R3e(2,3), R3e(1,3));
q5_a(2) = atan2(-R3e(3,1),-R3e(3,2)); %check why -ve sign is required









%end test


% if q1 > upperLim(1) || q1 <lowerLim(1) || q2 > upperLim(2) || q2 < lowerLim(2)
%     isPos = 0;
%     q = [0 0 0 0 0];
%     return 
% end 

theta3_cy = -pi/2 + acos((-a2^2 - a3^2 +(x_c^2 + y_c^2)+((d1-z_c)^2))/(2*a2*a3));
q3_a(4) = theta3_cy;
    
theta2_cy = pi/2 -atan2((z_c-d1),(sqrt(x_c^2 + y_c^2))) + atan2((a3*sin(-pi/2-q3_a(3))),(a2+a3*cos(-pi/2-q3_a(3))));
q2_a(5) = theta2_cy;

        
        
q  = [q1, q2_a(1), q3_a(1), q4_a(1), q5_a(1);
      q1, q2_a(2), q3_a(1), q4_a(1), q5_a(1);
      q1, q2_a(3), q3_a(1), q4_a(2), q5_a(2);
      q1, q2_a(4), q3_a(1), q4_a(2), q5_a(2);
      q1, q2_a(5), q3_a(2), q4_a(2), q5_a(2);
      q1, q2_a(6), q3_a(2), q4_a(2), q5_a(2);
      q1, q2_a(7), q3_a(2), q4_a(2), q5_a(2);
      q1, q2_a(8), q3_a(2), q4_a(2), q5_a(2);      
      q1, q2_a(5), q3_a(4), q4_a(1), q5_a(1); %cy_s value  
      q1, q2_a(2), q3_a(1), q4_a(2), q5_a(2);
];


isPos = true;





function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
hom_trans_matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];
end

%archival code

% alpha = atan2(z_c-d1, sqrt(x_c^2+y_c^2));
% beta = atan2(a3*c3_a, a2+a3*s3_a);
% q2_a(1) = pi/2 - alpha - beta(1);
% q2_a(2) = pi/2 - alpha - beta(2);
% q2_a(3) = pi/2-alpha+beta(1);
% q2_a(4) = pi/2-alpha+beta(2);
% q2_a(3) = 3*pi/2-alpha-beta(1);
% q2_a(4) = 3*pi/2-alpha-beta(2);



% q3 = q3_a(2);
% c3 = cos(q3); 
% s3 = sin(q3);
% q2 = q2_a(6);
% c2 = cos(q2);
% s2 = sin(q2);
% 
% % R3e = (R03).T * R0e 
% r3e_cos4 = r13*(c1*c2*c3 - c1*s2*s3) + r23*(c2*c3*s1 - s1*s2*s3) - r33*(c2*s3 + c3*s2);
% r3e_sin4 = -r13*(c1*c2*s3 + c1*c3*s2) - r23*(c2*s1*s3 + c3*s1*s2) - r33*(c2*c3 - s2*s3); 
% 
% q4 = atan2(r3e_sin4, r3e_cos4);
% q5 = atan2(r11*s1-r21*c1, r12*s1-r22*c1); 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end