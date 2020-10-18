
clc 
clear 

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
% for th1 = -1.4 : 0.2 :1.4
%     q1 = [th1,0,0,0,0,0];
%     
% 
%     q = q1;
%     w_1= calc_wrist_pos(a1,q);
% 
%     q_test = angle_ik(a1, w_1);
%     q;
%     q_test_con = [q_test_con;q,bl,q_test];
% end 


% for th1 = -1.4 : 0.2 :1.4
%    for th2 = -1.2 :0.5 : 1.4
%         for th3 = -1.8 : 0.3 : 1.7
%              q1 = [th1,th2,th3,0,0,0];
%     
% 
%                 q = q1;
%                 w_1= calc_wrist_pos(a1,q);
% 
%                 q_test = angle_ik(a1, w_1);
%                 q_test_con = [q_test_con;q,bl,q_test];
%         end
%    end
% end 

elements = 12;
% q_test = zeros(elements,3);
for th1 = -1.4 : 0.2 :1.4
   for th2 = -1.2 :0.1 : 1.4
        for th3 = -1.8 : 0.1 : 1.7
             q1 = [th1,th2,th3,0,0,0];
    

                q = q1;
                w_1= calc_wrist_pos(a1,q);
                
                q_test = [];
                for i = 1:elements
                    q_test = [q_test,angle_ik(a1, w_1, i)];
                end
                
                q_test_con = [q_test_con;q(:,1:3), q_test];                
                
        end
   end
end

q_test_con = round(q_test_con, 5);


q_test_con = [q_test_con, zeros(size(q_test_con,1),1), zeros(size(q_test_con,1),1), zeros(size(q_test_con,1),1)];
for i = 1:elements
   q_test_con(:,3*(elements+1)+1)= q_test_con(:,3*(elements+1)+1)+(q_test_con(:,1)==q_test_con(:,1+3*i));
   q_test_con(:,3*(elements+1)+2)= q_test_con(:,3*(elements+1)+2)+(q_test_con(:,2)==q_test_con(:,2+3*i));
   q_test_con(:,3*(elements+1)+3)= q_test_con(:,3*(elements+1)+3)+(q_test_con(:,3)==q_test_con(:,3+3*i));   
end

q_notfound = [sum(q_test_con(:,3*(elements+1)+1)==0),sum(q_test_con(:,3*(elements+1)+2)==0), sum(q_test_con(:,3*(elements+1)+3)==0), size(q_test_con, 1)];

q_theta2 = [];
q_theta1 = [];
for i = 1:9
    q_theta2 = [q_theta2, q_test_con(:,-1+3*i)];
    q_theta1 = [q_theta1, q_test_con(:,3*i)];
end

%single value test1
% q1 = [0, 1.7, -1.5, 0,0,0]
% q = q1;
% w_1= calc_wrist_pos(a1,q);
% q_test = angle_ik(a1, w_1)

function [w_value] = calc_wrist_pos(a, q)

    w_value =  [a(2)*cos(q(1))*sin(q(2)) - a(3)*cos(q(1))*sin(q(2))*sin(q(3)) + a(3)*cos(q(1))*cos(q(2))*cos(q(3));
                a(2)*sin(q(1))*sin(q(2)) - a(3)*sin(q(1))*sin(q(2))*sin(q(3)) + a(3)*sin(q(1))*cos(q(2))*cos(q(3));
                a(1) + a(2)*cos(q(2)) - a(3)*cos(q(2))*sin(q(3)) - a(3)*cos(q(3))*sin(q(2))];

end


function [q_value] = angle_ik(a, pos, option)
x_c = pos(1);
y_c = pos(2);
z_c = pos(3);

d1 = a(1);
a2 = a(2);
a3 = a(3);

lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

    %Calculating q1 
    q1 = atan2(y_c,x_c);
    % q1 = atan2(abs(y_c),abs(x_c));
    if q1==pi ||q1 == -pi
      q1=0; % To remove -pi and pi from answers
    end
   
 q1_a = [];
 
 elements = 12;
 for i=1:elements
    if i ==1
        q1_a(i) = q1;
    elseif i ==2;
         q1_a(i) = pi+q1;
%     elseif i ==3;
%          q1_a(i) = pi-q1;
%     elseif i ==4;
%          q1_a(i) = -q1;
    elseif i ==5;
         q1_a(i) = q1-pi;         
    else
        q1_a(i) = q1;
    end    
end


%Calculating q3
gamma = acos((a2^2 + a3^2 - x_c^2 - y_c^2 - (z_c-d1)^2) / (2*a2*a3));

%2 values for elbow up and elbow dowm orientation
q3_a(1) = pi/2-gamma;
q3_a(2) = gamma-3*pi/2 ;
q3_a(3)  = pi/2+gamma;



%Calculating q2
%Here we calculate 8 values 
%2 for elbow-up and elbow-down for +ve q2 and 2 for elbow-up and elbow-down for -ve q2 
%the 4 values are then calculated for each value of q3 
% We later discard the incorrect/out of range values
q2_a(1) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(2) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
% q2_a(3) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));
q2_a(3) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*sin(q3_a(1)),(a2-a3*cos(q3_a(1))));
q2_a(4) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(1)),(a2-a3*sin(q3_a(1))));

q2_a(5) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(6) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
% q2_a(7) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));
q2_a(7) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*sin(q3_a(1)),(a2-a3*cos(q3_a(1))));
q2_a(8) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(2)),(a2-a3*sin(q3_a(2))));

q2_a(9) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(3)),(a2-a3*sin(q3_a(3))));
q2_a(10) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(3)),(a2-a3*sin(q3_a(3))));
% q2_a(11) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_a(3)),(a2-a3*sin(q3_a(3))));
q2_a(11) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*sin(q3_a(1)),(a2-a3*cos(q3_a(1))));
q2_a(12) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_a(3)),(a2-a3*sin(q3_a(3))));



% 
% q2_a(1) = 0;
% q2_a(5) = 0;
% q2_a(9) = 0;


%Constructing q3 vector to match the size of q2 vector
q3_b = [];
for i=1:elements
    if i <= 4 
        q3_b(i) = q3_a(1);
    elseif i >4 && i <=8
        q3_b(i) = q3_a(2);
    elseif i >8 && i<=12
        q3_b(i) = q3_a(3);
    else
        q3_b(i) = 0;
    end    
end

    
    q_value = [q1_a(option), q2_a(option), q3_b(option)];
end
        