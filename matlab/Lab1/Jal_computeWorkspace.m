addpath('../Core') % references ROS interface and arm controller files you'll need for every lab


gripper = [];
q_concat = [];

for th_1  = -1.4 :0.1 :1.4
    for th_2 = -1.2 :0.1 : 1.4
        for th_3 = -1.8 : 0.1: 1.7
            for th_4 = -1.9 : 0.1 : 1.7
                for th_5 = -2 : 0.1 : 1.5
                        
                        q = [th_1, th_2, th_3, th_4, th_5, 0];
                        [jointPositions, T0e] = abby_calculateFK(q);
                        gripper = [gripper; T0e([13,14,15])];
%                         q_concat = [q_concat;q];
%                             disp(T0e);
                        
                    
                end
                endx
        end
    end                
end;


% disp(gripper)
scatter3(gripper(:,1), gripper(:,2), gripper(:,3))