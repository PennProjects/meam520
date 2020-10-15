classdef ArmController
    properties     
        % set "true" if you want to use gripper mode, "false" change to lidar mode
        enableGripper
        
        % ros node
        node
        
        % subscribers
        state_sub  
        pose_sub
        lidar_sub
        
        % publishers    
        pos_pubs = [];
        gripper_pubs = [];
        lidar_pub
        
        % c*6 a series of command for the Lynx arm
        state_command

        % 1*6 Joint variables for the Lynx arm
        cur_state
        cur_vel
        cur_pose
        
        lidar
        
        transforms = {};
        jointPositions = zeros(6,3);

        joint_limits = [ -1.4, -1.2, -1.8, -1.9, -2.0, -15;
                          1.4,  1.4,  1.7,  1.7,  1.5,  30]; 
                      
        JOINT_NAMES = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"];
    end
    
    methods
        
        %% constructor %%
        function obj = ArmController(enableGripper)
            
             % Initialization
             % Any code not using output argument (obj)
             
             obj.enableGripper = enableGripper;
             disp("[INFO]: Starting ROS node...");
             
             % intial the node
             obj.node = ros.Node('/arm_controller');
             
             % intial the subscriber;
             obj.state_sub = rossubscriber('/joint_states');
             
             obj.pose_sub = {}; 
             
             for i = 1:6
                 name = '/joint_poses/'+obj.JOINT_NAMES(i);
                 obj.pose_sub{i} = rossubscriber(name);
             end 

             % intial the publisher
             obj = intial_pub(obj);

             
             % update position
             obj = update_state(obj);
            
        end
        
        
        %% intial the publisher 
        function obj = intial_pub(obj)    
            for i = 1:5
                % pub = ros.Publisher(node,topicname,type)
                pub = ros.Publisher(obj.node, "/al5d_arm_position_controller"+num2str(i)+"/command", 'std_msgs/Float64');                     
                
                obj.pos_pubs = [obj.pos_pubs; pub];
            end
            
            if obj.enableGripper
                for i=1:2
                    pub = ros.Publisher(obj.node, "/al5d_gripper_controller"+num2str(i)+"/command", 'std_msgs/Float64');
                    obj.gripper_pubs = [obj.gripper_pubs; pub];
                end
            else
                obj.lidar_pub = ros.Publisher(obj.node, "/al5d_lidar_controller/command", 'std_msgs/Float64');
                
                % change the limitations
                obj.joint_limits(1,5) = -1.4;
                obj.joint_limits(2,5) = 1.4;
                
                % subscribe the laser scan
                obj.lidar_sub = rossubscriber('/scan');
                
            end
            
               
        end
        
        
        %% add command to the lynx arm
        function obj = add_command(obj,paths)

            
            disp('[INFO]: Current state value is ...');
            disp(obj.cur_state);
            
            
            [c,r] = size(paths);
            
            disp('[INFO]: Start send message ...');
            
            if r == 6 %for each joint
                obj.state_command = paths;             
                % check command values
                for i = 1:c
                    for j = 1:6
                        if obj.state_command(i,j) < obj.joint_limits(1,j)
                            warning("[WARN]: State " + num2str(j) + " is below the limit " + num2str(obj.joint_limits(1,j)));
                            obj.state_command(i,j) = obj.joint_limits(1,j);
                        end
                        if obj.state_command(i,j) > obj.joint_limits(2,j)
                            warning("[WARN]: State " + num2str(j) + " is above the limit " + num2str(obj.joint_limits(2,j)));
                            obj.state_command(i,j) = obj.joint_limits(2,j);
                        end                  
                    end
                    disp('[INFO]: The command is ...');
                    disp(obj.state_command(i,:));
                    obj.state_command(i,6) = (-obj.state_command(i,6)+30.)/45.*0.03;
                    %obj = calibration(obj,obj.state_command(i,:));
                    obj = linear_interpolate(obj,obj.state_command(i,:));
                    pause(0.1);
                    
                end  
            else
                disp('[ERROR]: Invalid state command !');
                rosshutdown;
            end
            % clear the state command
            obj.state_command = [];
            obj = update_transforms(obj);
        end

        
        
        %% linear interpolation from the current state to the command state
        function obj =  linear_interpolate(obj,command)
            diff =  command-obj.cur_state;
            curr = obj.cur_state;                
            dist = max(abs(diff));        
            N = max(round(dist*30),3);
            if N<1
                 obj = update_move(obj, command);
            else                
                new_command = curr + transpose(linspace(1, N, N))*diff*(1/N);
                obj = update_move(obj, new_command);
         
            end
            
            obj = calibration(obj,command);
        end
            
           
       
        %% update movements
        function obj = update_move(obj,new_command)
            
            % create the message type for publisher
            if ~obj.enableGripper
                obj.measure_lidar();
            end
            msg = rosmessage('std_msgs/Float64'); 
            

            
            [c,~] = size(new_command);
            
            
            for j = 1:c
                command = new_command(j,:);
%                 disp("current command is ...");
%                 disp(command);
%                 disp("current state is ...");
%                 disp(obj.cur_state);                
                for i = 1:5                                       
                    msg.Data = command(1,i);
                    send(obj.pos_pubs(i),msg);
                end

                if obj.enableGripper
                    % set left gripper
                    msg.Data = -command(1,6);                
                    send(obj.gripper_pubs(1), msg);
                    % set right gripper
                    msg.Data = command(1,6);
                    send(obj.gripper_pubs(2), msg);
                    
                else
                    % set lidar
                    msg.Data = command(1,6);
                    send(obj.lidar_pub, msg);
                end
                obj = update_state(obj); 
%                 pause(0.1);
            end
                   
        end
        
        %% fake calibration
        function obj = calibration(obj,command)
            msg = rosmessage('std_msgs/Float64'); 
            iteration = 0;
            while sum(abs(obj.cur_state - command)) > 0.02 && iteration<15
%                  disp("current command is ...");
%                  disp(command);
%                  disp("current state is ...");
%                  disp(obj.cur_state);
                 for i = 1:5                                       
                     msg.Data = command(1,i);
                     send(obj.pos_pubs(i),msg);
                 end

                 if obj.enableGripper
                     % set left gripper
                     msg.Data = -command(1,6);                
                     send(obj.gripper_pubs(1), msg);
                     % set right gripper
                     msg.Data = command(1,6);
                     send(obj.gripper_pubs(2), msg);
                 else
                     msg.Data = command(1,6);
                     send(obj.lidar_pub, msg);
                 end      
                  obj = update_state(obj);
                  %disp(obj.cur_state);
                  iteration = iteration +1;
            end
            disp("[INFO]: Finish calibration ... ")
            disp("[INFO]: Now the position is: ... ");
            disp(obj.cur_state);   
        
        end
        
        
        %% use the subscriber to update current state and velocity
        function obj = update_state(obj)
            state = receive(obj.state_sub,10);
            %disp(state.Position)
            if obj.enableGripper
                obj.cur_state = [state.Position(1), state.Position(5), state.Position(2), ...
                        state.Position(7), state.Position(6), state.Position(3)];    	
                obj.cur_vel = [state.Velocity(1), state.Velocity(5), state.Velocity(2), ...
                        state.Velocity(7), state.Velocity(6), state.Velocity(3)];    
            else
                obj.cur_state = [state.Position(1), state.Position(4), state.Position(2), ...
                        state.Position(6), state.Position(5), state.Position(3)];                      
                obj.cur_vel = [state.Velocity(1), state.Velocity(4), state.Velocity(2), ...
                        state.Velocity(6), state.Velocity(5), state.Velocity(3)];                      
            end
        end   
        
        
        function obj = update_transforms(obj)
            for i = 1:6
                trans = receive(obj.pose_sub{i},10);
                translation = 1000*[trans.Transform.Translation.X,trans.Transform.Translation.Y,trans.Transform.Translation.Z];
                
                obj.jointPositions(i,1:3) = translation;
                
                quat = quaternion(trans.Transform.Rotation.W,...
                                           trans.Transform.Rotation.X,...
                                           trans.Transform.Rotation.Y,...
                                           trans.Transform.Rotation.Z);
%                 quat = quaternion(trans.Transform.Rotation.X,...
%                                            trans.Transform.Rotation.Y,...
%                                            trans.Transform.Rotation.Z,...
%                                            trans.Transform.Rotation.W);
                                       
                                       
                Ti = eye(4,4);
                
                rotationMatrix = rotmat(quat,'point');
               
                Ti(1:3,1:3) = rotationMatrix;
                Ti(1:3,4) = translation;
                obj.transforms{i} = Ti;
            end
  
        end
        
        
        function [jointPositions_sim,T0e_sum] = checkFK(obj)
        
            T0e_sum = obj.transforms{6};
            
            jointPositions_sim = obj.jointPositions;
            
        
        end
        
         
         
        %% delete the node
        function delete(obj)          
            delete(obj.node);
            %to delete the node so that there will not be some disputes of
            %nodes with same names
        end

        
        %% receive lidar message and do some measurements
        function obj= measure_lidar(obj)
            
            obj.lidar = receive(obj.lidar_sub,10);
            
            Ranges = obj.lidar.Ranges; % a list of ranges
            AngleMin = obj.lidar.AngleMin;
            AngleMin = obj.lidar.AngleMax;
            % can fulfill more
            plot(Ranges);
            
        end
            
        

        
    end
    
    
end