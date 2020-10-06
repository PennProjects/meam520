
syms c1 s1 d1 c2 s2 a2 c3 s3 a3 c4 s4 d4 c5 s5 d5
H01 = [ c1 0 -s1 0;
        s1 0 c1 0;
        0 -1 0 d1;
        0 0 0 1];
    
H12 = [s2 c2 0 a2*s2;
      -c2 s2 0 -a2*c2;
      0 0 1 0;
      0 0 0 1];
  
  
H23 = [-s3 -c3 0 -a3*s3;
       c3 -s3 0 a3*c3;
       0 0 1 0;
       0 0 0 1];
   
H34 = [s4 0 c4 0;
       -c4 0 s4 0;
        0 -1 0 0 ;
        0 0 0 1];
H4e = [c5 -s5 0 0;
       s5 c5 0 0;
       0 0 1 d4+d5;
       0 0 0 1];
   
H03 = H01*H12*H23;
H3e = H34*H4e;

% H0e = H03*H3e

% H3e_1 = transpose(H03)*H0e
% w = H03([13 14 15]);
% a = H03(1:3,5:7,9:11)





a1 = [76.2, 146.05, 187.325];
% q1 = [0,0,0,0,0,0];
% q2 = [-0.986, 0, 0.123, 0,1.5,0];
% q3 = [pi/2, 0, pi/4,0,pi/2,0];
% q4 = [0, 1.5, 1.2, 0,0,0];
q5 = [0, 0.3456, -1.8, 0,0,0];
q5_m = [0, 0.0879, -1.3416, 0, 0, 0];
%rotating only theta1
q_test_con = [];
bl = [];
for th1 = -1.4 : 0.2 :1.4
    q1 = [th1,0,0,0,0,0];
    

    q = q1;
    w_1= calc_wrist_pos(a1,q);

    q_test = angle_ik(a1, w_1);
    q;
    q_test_con = [q_test_con;q,bl,q_test];
end   


function [w_value] = calc_wrist_pos(a, q)

    w_value =  [a(2)*cos(q(1))*sin(q(2)) - a(3)*cos(q(1))*sin(q(2))*sin(q(3)) + a(3)*cos(q(1))*cos(q(2))*cos(q(3));
                a(2)*sin(q(1))*sin(q(2)) - a(3)*sin(q(1))*sin(q(2))*sin(q(3)) + a(3)*sin(q(1))*cos(q(2))*cos(q(3));
                a(1) + a(2)*cos(q(2)) - a(3)*cos(q(2))*sin(q(3)) - a(3)*cos(q(3))*sin(q(2))];

end


function [q_value] = angle_ik(a, pos)

    theta1 = atan2(pos(2),pos(1));
    
    omega = asin((a(2)^2+ a(3)^2 -(pos(1)^2 + pos(2)^2)-((a(1)-pos(3))^2))/(2*a(2)*a(3)));
    
%     theta3 = omega;
%     theta3 = -omega-pi;
    
    
    
    w = ((a(2)^2+ a(3)^2 -(pos(1)^2 + pos(2)^2)-((a(1)-pos(3))^2))/(2*a(2)*a(3)));
    sqrt(1-w^2);
    theta3 = atan2(w,sqrt(1-w^2)); %up elbow
%     theta3 = atan2(w,-1*sqrt(1-w^2)) ; % elbow down
    
    theta3_cy = -pi/2 + acos((-a(2)^2 - a(3)^2 +(pos(1)^2 + pos(2)^2)+((a(1)-pos(3))^2))/(2*a(2)*a(3)));
    
    if pos(1) < 0 || pos(2) <0 
        alpha = -1* atan2((sqrt(pos(1)^2 + pos(2)^2)),(a(1)-pos(3)));
    else
        alpha = +1* atan2((sqrt(pos(1)^2 + pos(2)^2)),(a(1)-pos(3)));
    end
    beta = atan2((a(3)*cos(theta3)),(a(2)-a(3)*sin(theta3)));
%     beta = pi/2;
%     alpha_2 = +1* atan2(-1*(a(1)-pos(3)), (sqrt(pos(1)^2 + pos(2)^2)))*180/pi
%     beta_2 = atan2((a(3)*cos(theta3)),(a(2)+a(3)*sin(theta3)))*180/pi
%     a_deg = alpha*180/pi
%     b_deg = beta*180/pi
    theta2 = pi-alpha-beta;
%     theta2 = pi+alpha+beta;

    
    q_value = [theta1 theta2 theta3];
end
        