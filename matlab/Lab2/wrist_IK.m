
% A program to test the IK values for wrist

clc 
clear


%Link lengths
a1 = [76.2, 146.05, 187.325];

%set of q generated
theta_sets = 36;


%Looping through all qs in range to verify IK
q_test_con = [];


% for th1 = -1.4 : 0.2 :1.4
 for th1 = 0 : 1 :0
   for th2 = -1.2 :0.2 : 1.4
        for th3 = -1.8 : 0.2 : 1.7
             q1 = [th1,th2,th3,0,0,0];
    
                q = q1;
                w_1= calc_wrist_pos(a1,q);
                
                q_test = [];
                for i = 1:theta_sets
                    q_test = [q_test,angle_ik(a1, w_1, i)];
                end
                
                q_test_con = [q_test_con;q(:,1:3), q_test];                
                
        end
   end
end

q_test_con = round(q_test_con, 5);

%checking if input thetas match any of the calculated sets
q_test_con = [q_test_con, zeros(size(q_test_con,1),1), zeros(size(q_test_con,1),1), zeros(size(q_test_con,1),1)];
for i = 1:theta_sets
   q_test_con(:,3*(theta_sets+1)+1)= q_test_con(:,3*(theta_sets+1)+1)+(q_test_con(:,1)==q_test_con(:,1+3*i));
   q_test_con(:,3*(theta_sets+1)+2)= q_test_con(:,3*(theta_sets+1)+2)+(q_test_con(:,2)==q_test_con(:,2+3*i));
   q_test_con(:,3*(theta_sets+1)+3)= q_test_con(:,3*(theta_sets+1)+3)+(q_test_con(:,3)==q_test_con(:,3+3*i));   
end

% caulating numbers of inputs for which the IK did not match the input
q_notfound = [sum(q_test_con(:,3*(theta_sets+1)+1)==0),sum(q_test_con(:,3*(theta_sets+1)+2)==0), sum(q_test_con(:,3*(theta_sets+1)+3)==0), size(q_test_con, 1)];
q_found_percentage = 100*[((q_notfound(4)-q_notfound(1))/q_notfound(4)), ((q_notfound(4)-q_notfound(2))/q_notfound(4)), ((q_notfound(4)-q_notfound(3))/q_notfound(4))]



%Function to find wrist position based on q input
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
 phi = atan2(y_c,x_c);
 
 if phi==pi ||phi == -pi
      phi=0; % To remove -pi and pi from answers
 end
   
q1_a(1) = phi;
q1_a(2) = phi+pi;      
q1_a(3) = phi-pi;


%Calculating q3
gamma = acos((a2^2 + a3^2 - x_c^2 - y_c^2 - (z_c-d1)^2) / (2*a2*a3));

%2 values for elbow up and elbow dowm orientation
q3_a(1) = pi/2-gamma;
q3_a(2) = gamma-3*pi/2 ;
q3_a(3)  = pi/2+gamma;



%Calculating q2
%Here we calculate 4 values 
%2 for elbow-up and elbow-down for +ve q2 and 2 for elbow-up and elbow-down for -ve q2 

    function [q2_a] = calc_q2(q3_val)
        q2_a(1) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_val(1)),(a2-a3*sin(q3_val(1))));
        q2_a(2) = pi/2-atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_val(1)),(a2-a3*sin(q3_val(1))));
        q2_a(3) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))-atan2(a3*cos(q3_val(1)),(a2-a3*sin(q3_val(1))));
        q2_a(4) = -pi/2+atan2((z_c-d1), sqrt(x_c^2+y_c^2))+atan2(a3*cos(q3_val(1)),(a2-a3*sin(q3_val(1))));
    end


%Constructing q1,q2,q3 matrix for all combinations
q1 = [];
q2 = [];
q3 = [];

q1_a_size = size(q1_a,2);
q2_a_size = 4;
q3_a_size = size(q3_a,2);


for i = 1:q1_a_size
    for k = 1:q3_a_size
        
        q2 = [q2;transpose(calc_q2(q3_a(k)))];
        
        for j = 1:q2_a_size
            q3 = [q3;q3_a(k)]; 
            q1 = [q1;q1_a(i)];
        end            
    end
end
    q_value = [q1(option), q2(option), q3(option)];
end