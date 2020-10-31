clc 
clear

syms theta1 theta2 theta3 theta4 theta5 L1 L2 L3 L4 L5;

theta1 = theta1 + pi;
theta2 = theta2 + pi/2;
theta3 = theta3 + pi/2;
theta4 = theta4 + pi/2;
theta5 = theta5 + pi;

A1 = DHParam(0, pi/2, L1, theta1);
A2 = DHParam(L2, 0, 0, theta2);
A3 = DHParam(L3, 0, 0, theta3);
A4 = DHParam(0, pi/2, 0, theta4);
A5 = DHParam(0, 0, L4+L5, theta5);

R01 = A1;
R02 = A1*A2;
R03 = A1*A2*A3;
R04 = A1*A2*A3*A4;
R0e = A1*A2*A3*A4*A5;

R01 = R01([1, 5, 9; 2, 6, 10; 3, 7, 11]);
R02 = R02([1, 5, 9; 2, 6, 10; 3, 7, 11]);
R03 = R03([1, 5, 9; 2, 6, 10; 3, 7, 11]);
R04 = R04([1, 5, 9; 2, 6, 10; 3, 7, 11]);
R0e = R0e([1, 5, 9; 2, 6, 10; 3, 7, 11]); 

O0e = A1*A2*A3*A4*A5(:,end);
O1e = A2*A3*A4*A5(:, end);
O2e = A3*A4*A5(:, end);
O3e = A4*A5(:, end);
O4e = A5(:, end);

O0e = O0e([1, 2, 3]);
O1e = O1e([1, 2, 3]);
O2e = O2e([1, 2, 3]);
O3e = O3e([1, 2, 3]);
O4e = O4e([1, 2, 3]);

Jw1 = [0; 0; 1];
Jv1 = SkewMat(Jw1)*O0e;

Jw2 = R01*[0; 0; 1];
Jv2 = SkewMat(Jw2)*R01*O1e;

Jw3 = R02 * [0; 0; 1];
Jv3 = SkewMat(Jw3)*R02*O2e;
SkewMat(Jw3)

Jw4 = R03 * [0; 0; 1];
Jv4 = SkewMat(Jw4)*R03*O3e;

Jw5 = R04*[0; 0; 1]
Jv5 = SkewMat(Jw5)*R04*O4e
SkewMat(Jw5)
Jv = [Jv1 Jv2 Jv3 Jv4 Jv5];

function [s_matrix] = SkewMat(z)
    ax = z(1);
    ay = z(2);
    az = z(3);
    s_matrix = [0 -az ay; 
                az 0 -ax;
                -ay ax 0];
end


function [hom_trans_matrix] = DHParam(a, alpha, d, theta)
%    Calculates DHParams when given four parameters
    if cos(alpha) > 0 && cos(alpha) < 1
        cos_alpha = 0;
    else
        cos_alpha = cos(alpha);
    end
    
        hom_trans_matrix = [cos(theta), -sin(theta)*cos_alpha, sin(theta)*sin(alpha), a*cos(theta); 
                            sin(theta), cos(theta)*cos_alpha, -cos(theta)*sin(alpha), a*sin(theta);
                            0, sin(alpha), cos_alpha, d;
                            0, 0, 0, 1];
    end