
joint_position_curr = [0 0 0 0 0 0];
for i = 2:6 
    J_ = zeros(6,(i-1))+ calcJacobian(joint_position_curr, i)
   
    Jv = J_(1:3,:);
    Jv(:,end+1:6) = 0;
%     tau = tau + Jv'*F(i,:)';
% %     Jv = [Jv;J_ ];
end

