
syms c1 s1 d1 c2 s2 a2 c3 s3 a3
H01 = [ c1 0 -s1 0;
        s1 0 c1 0;
        0 -1 0 d1;
        0 0 0 1]
    
H12 = [s2 c2 0 a2*s2;
      -c2 s2 0 -a2*c2;
      0 0 1 0;
      0 0 0 1]
  
  
H23 = [-s3 -c3 0 -a3*s3;
       c3 -s3 0 a3*c3;
       0 0 1 0;
       0 0 0 1]
   
H03 = H01*H12*H23

w = H03([13 14 15])





a1 = [76.2, 146.05, 187.325];
% q1 = [0,0,0,0,0,0];
q2 = [-1, 0, 0.2, 0,1.5,0];
% q3 = [pi/2, 0, pi/4,0,pi/2,0];

q = q2;
w_1= calc_wrist_pos(a1,q)

q_test = angle_ik(a1, w_1)
q


function [w_value] = calc_wrist_pos(a, q)

    w_value =  [a(2)*cos(q(1))*sin(q(2)) - a(3)*cos(q(1))*sin(q(2))*sin(q(3)) + a(3)*cos(q(1))*cos(q(2))*cos(q(3));
                a(2)*sin(q(1))*sin(q(2)) - a(3)*sin(q(1))*sin(q(2))*sin(q(3)) + a(3)*sin(q(1))*cos(q(2))*cos(q(3));
                a(1) + a(2)*cos(q(2)) - a(3)*cos(q(2))*sin(q(3)) - a(3)*cos(q(3))*sin(q(2))];

end


function [q_value] = angle_ik(a, pos)
    
    q_value = [atan2(pos(2),pos(1)),
               0,
               asin((a(2)^2+ a(3)^2 -(((pos(1)^2 + pos(2)^2)/pos(2)^2)*(pos(2)^2+(a(1)-pos(3))^2)))/(2*a(2)*a(3)))];
end
        