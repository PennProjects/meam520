d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 68;                        % Distance between joint 4 and end effector

q1 = atan2(y_c,x_c);
q2 = ;
q3 = ; 

c1 = cos(q1); s1 = sin(q1);
c2 = cos(q2); s2 = sin(q2);
c3 = cos(q3); s3 = sin(q3); 

% H03 = [ c1*c2*c3 - c1*s2*s3, - c1*c2*s3 - c1*c3*s2, -s1, a2*c1*s2 - a3*c1*s2*s3 + a3*c1*c2*c3;
%         c2*c3*s1 - s1*s2*s3, - c2*s1*s3 - c3*s1*s2,  c1,    a2*s1*s2 - a3*s1*s2*s3 + a3*c2*c3*s1;
%         - c2*s3 - c3*s2,         s2*s3 - c2*c3,         0,     d1 + a2*c2 - a3*c2*s3 - a3*c3*s2;
%                    0,                     0,            0,                                    1];
% O03_x = H03(1,4);
% O03_y = H03(2,4);
% O03_z = H03(3,4);


origin = [0 0 0];
one_point = [0 0 d1]; 
joint_angles = [q1, q2, q3];
wrist_center = calc_wrist_pos([d1 a2 a3], joint_angles);

[a, b, c, d] = get_plane_eq([0 0 0], one_point, wrist_center.');


function [w_value] = calc_wrist_pos(a, q)

    w_value =  [a(2)*cos(q(1))*sin(q(2)) - a(3)*cos(q(1))*sin(q(2))*sin(q(3)) + a(3)*cos(q(1))*cos(q(2))*cos(q(3));
                a(2)*sin(q(1))*sin(q(2)) - a(3)*sin(q(1))*sin(q(2))*sin(q(3)) + a(3)*sin(q(1))*cos(q(2))*cos(q(3));
                a(1) + a(2)*cos(q(2)) - a(3)*cos(q(2))*sin(q(3)) - a(3)*cos(q(3))*sin(q(2))];

end

function [ a, b, c, d ] = get_plane_eq( p1, p2, p3 )

  a = ( p2(2) - p1(2) ) * ( p3(3) - p1(3) ) ...
    - ( p2(3) - p1(3) ) * ( p3(2) - p1(2) );

  b = ( p2(3) - p1(3) ) * ( p3(1) - p1(1) ) ...
    - ( p2(1) - p1(1) ) * ( p3(3) - p1(3) );

  c = ( p2(1) - p1(1) ) * ( p3(2) - p1(2) ) ...
    - ( p2(2) - p1(2) ) * ( p3(1) - p1(1) );

  d = - p2(1) * a - p2(2) * b - p2(3) * c;

  return
end

function [onPlane] = check_on_plane( ori, a, b, c, d)
    if a*ori(1) +b*ori(2)+c*ori(3)+d == 0
        onPlane = 1;
    else
        onPlane = 0;
    end
    return
end


