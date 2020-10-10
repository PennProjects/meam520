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
d5 = 68;                        % Distance between joint 4 and end effector

% Joint limits
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

%% Your code here
% Decomposing T0e
%rotation matrix of end effector target
r11 = T0e(1,1);
r12 = T0e(1,2);
r13 = T0e(1,3);
r21 = T0e(2,1);
r22 = T0e(2,2);
r23 = T0e(2,3);
r31 = T0e(3,1);
r32 = T0e(3,2);
r33 = T0e(3,3);
%position of end effector target
x = T0e(1,4);
y = T0e(2,4);
z = T0e(3,4);

% wrist center positions
x_c = x - (d5) * r13; 
y_c = y - (d5) * r23;
z_c = z - (d5) * r33;
o_c = [x_c; y_c; z_c]; % Position of wrist centre to reach

% link vector
L = [d1,a2,a3,d4,d5];



%Calculating q1 
q1 = atan2(y_c,x_c);
% q1 = atan2(abs(y_c),abs(x_c));
if q1==pi ||q1 == -pi
    q1=0; % To remove -pi and pi from answers
end


%Calculating q3
gamma = acos((a2^2 + a3^2 - x_c^2 - y_c^2 - (z_c-d1)^2) / (2*a2*a3));

%2 values for elbow up and elbow dowm orientation
q3_a(1) = pi/2-gamma;
q3_a(2) = gamma-3*pi/2 ;



%Calculating q2
%Here we calculate 8 values 
%2 for elbow-up and elbow-down for +ve q2 and 2 for elbow-up and elbow-down for -ve q2 
%the 4 values are then calculated for each value of q3 
% We later discard the incorrect/out of range values
q2_a(1) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(2) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(3) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(4) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));

q2_a(5) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(6) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(7) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(8) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));


%Constructing q3 vector to match the size of q2 vector
q3_b = [];
for i=1: 8
    if i <= 4
        q3_b(i) = q3_a(1);
    elseif i >=5 && i <=8
        q3_b(i) = q3_a(2);
    end    
end



%Calculation of q4 and q5 for all possible values of q1,q2 and q3
q = [];
for i = 1: size(q2_a, 2)
    
    [tmp_list] = gen_theta4_5(T0e, q1, q2_a(i), q3_b(i), L);
    q = [q; tmp_list];
end



%Checking if the angles are within limits
within_limit_flag = ones*[1:size(q,1)];

for i =1:size(q,1)
    if q(i,1) > upperLim(1) || q(i,1) < lowerLim(1)
       within_limit_flag(i) = 0;
    elseif q(i,2) > upperLim(2) || q(i,2) < lowerLim(2)
        within_limit_flag(i) = 0 ;
    elseif q(i,3) > upperLim(3) || q(i,3) < lowerLim(3)
        within_limit_flag(i) = 0;
    elseif q(i,4) > upperLim(4) || q(i,4) < lowerLim(4)
        within_limit_flag(i) = 0;
    elseif q(i,5) > upperLim(5) || q(i,5) < lowerLim(5)
        within_limit_flag(i) = 0;
    end   
end

%Inidicating target transformation is located outside the reachable workspace
if sum(within_limit_flag)==0
    isPos = 0;
else
    isPos = 1;
end

%Removing q values that are outside the reachable workspace
q_reduced = [];
for i =1:size(q,1)
    if within_limit_flag(i)
        q_reduced = [q_reduced;q(i,:)];
    end
end

% % % % % % 
% Now we check if there target transformation is feasible
%3 points in the plane of the robot
% origin, a point along z axis and the wirst centre calculated above
p1 = [0 0 0];
p2 = [0 0 50];
p3 = transpose(o_c);

%Finding the normal vector for the robot plane
%this plane is formed by the points p1, p2 and p3
normal_robot_plane = cross(p1-p2,p1-p3);
normal_robot_plane = 1*normal_robot_plane/norm(normal_robot_plane);

%Checking if the target is feasible
%The target is feasible if it lies in the robot plane
e_desired = [x,y,z];

%This the the vector from the wrist centre to e_desired
e_desired_from_wrist = e_desired-transpose(o_c); 
e_proj_on_normal = dot(e_desired_from_wrist,normal_robot_plane)*normal_robot_plane;

% e_possible is the projection of e_desired vector from wrist in the robot plane
e_possible_from_wrist =  e_desired_from_wrist-e_proj_on_normal; 

%we add the wrist_centre to get e_possible in global frame
e_possible = e_possible_from_wrist+transpose(o_c);

%calculating the angle between the normal to the robot plane and the e_desired vector
%If this value is 0, e desired is feasible and e_desired = e_possible
e_norm = e_desired/norm(e_desired);
feasibility_check = round(dot(normal_robot_plane, e_norm),3);

if feasibility_check ~= 0
    isPos = 0;
end
% % % % % % % % 

%Now we get the rotation matrix of e_possible
%zaxis of the e_desired is projected onto the robot plane
e_desired_zaxis_unitvector = [r13,r23,r33]; %As given in T0e
e_desired_zaxis_unitvector_proj_on_normal = dot(e_desired_zaxis_unitvector,normal_robot_plane)*normal_robot_plane;
%we now get the z axis of the e_possible  = projection of e_desired on robot plane
e_possible_zaxis_vector = e_desired_zaxis_unitvector-e_desired_zaxis_unitvector_proj_on_normal;
e_possible_zaxis_norm = e_possible_zaxis_vector/norm(e_possible_zaxis_vector); % normalizing z axis of e_possible




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


q = q_reduced;
isPos

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end