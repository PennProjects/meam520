function [q isPos] = calculateIK(T0e)
%T0e = [[   0.019,    0.969,    0.245,   47.046];[   0.917,   -0.115,    0.382,   73.269];[   0.398 ,   0.217,   -0.891,  100.547];[   0.,       0. ,      0.,       1.]];

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
o_c = [x_c; y_c; z_c];

% links
L = [d1,a2,a3,d4,d5];

% define qs
% define q1
q1 = atan2(y_c,x_c);
if q1==pi
    q1=0;
end

% define q3
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

q3_b = [];
for i=1: 8
    if i <= 4
        q3_b(i) = q3_a(1);
    elseif i >=5 && i <=8
        q3_b(i) = q3_a(2);
    end    
end

q = [];
for i = 1: size(q2_a, 2)
    
    [tmp_list] = gen_theta4_5(T0e, q1, q2_a(i), q3_b(i), L);
    q = [q; tmp_list];
end

% iterate through qs and sort by distance 
% end_effector position through FK 
for i =1:size(q,2)
    H01 = DHParam(0, -pi/2, L(1), q(i,1));
    H12 = DHParam(L(2), 0, 0, q(i,2)-pi/2);
    H23 = DHParam(L(3), 0, 0, q(i,3)+pi/2);
    H34 = DHParam(0, -pi/2, 0, q(i,4)-pi/2);
    H4e = DHParam(0, 0, L(4)+L(5), q(i,5));
    H0e = H01 * H12 * H23 * H34 * H4e;
    wrist_ori = H0e(1:3, 4);
    if q(i,1) > upperLim(1) || q(i,1) < lowerLim(1)
        q(i,:)
        q(i,:)=[]; % remove from q 
    elseif q(i,2) > upperLim(2) || q(i,1) < lowerLim(2)
        q(i,:)
        q(i,:)=[]; % remove from q 
    elseif q(i,3) > upperLim(3) || q(i,1) < lowerLim(3)
        q(i,:)
       q(i,:)=[]; % remove from q 
    elseif q(i,2) > upperLim(4) || q(i,1) < lowerLim(4)
        q(i,:)
        q(i,:)=[]; % remove from q 
    elseif q(i,2) > upperLim(5) || q(i,1) < lowerLim(5)
        q(i,:)
        q(i,:)=[]; % remove from q 
    end
    
end



% check if it is in reachable workspace
% from wc to (0, 0, d1)
wrist_to_d1 = sqrt(x_c^2 + y_c^2 + (z_c-d1)^2);
if (wrist_to_d1 > a3 + a2 || wrist_to_d1 < a3-a2)
    isPos = 0; 
    q = [];
    return
end

isPos = 1;

function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
hom_trans_matrix = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos(alpha), d;
                            0, 0, 0, 1];
end

function [q_list] = gen_theta4_5(T0e_, theta1, theta2,theta3, L_)
         T01_ = DHParam(0, -pi/2, L_(1), theta1);
         T12_ = DHParam(L_(2), 0, 0, theta2-pi/2);
         T23_ = DHParam(L_(3), 0, 0, theta3+pi/2);
         T03_ = T01_ * T12_ * T23_;
         R03_ = T03_(1:3, 1:3);
         R0e_ = T0e_(1:3,1:3);
         R3e_ = transpose(R03_)*R0e_;
           
         q4 = atan2(R3e_(2,3), R3e_(1,3));
         q5 = atan2(-R3e_(3,1),-R3e_(3,2));
         
         q_list = [theta1, theta2, theta3, q4, q5];
         
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end