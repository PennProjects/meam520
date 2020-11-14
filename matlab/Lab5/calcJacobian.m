function J = calcJacobian(q, joint)
% CALCJACOBIAN Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration. Uses specific methods.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [0,6] representing which joint we care about
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%
% AUTHOR
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu) 10/25/19 
%       Modified from calcJacobianAllJoints_sol, by Dr. Cynthia Sung

%% Check Valid Input

J = [];

if nargin < 2
    return
elseif joint <= 1 || joint > 6
    return
end

%% Initialize
% relevant info from position FK
[jointPositions,T0] = calculateFK(q);

%% ANGULAR VELOCITY JACOBIAN
Jw1 = zeros(3,joint - 1);
for i = 1:(joint - 1)
    Jw1(:,i) = T0(1:3,3,i);
end

%% LINEAR VELOCITY JACOBIAN
Jv2 = zeros(3,joint - 1);
for i = 1:(joint - 1)
	Jv2(:,i) = cross(T0(1:3,3,i), jointPositions(joint,:)'-T0(1:3,4,i));
end

%% Compose Jacobian Matrix and calc end effector velocities
J = [Jv2; Jw1];  


end