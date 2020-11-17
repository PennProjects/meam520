classdef ArmController < handle
        
    properties (Access = private)    
                
        % subscribers
        joint_sub  
        pose_sub
        collided_sub  
        model_sub
        opponent_sub
        
        % publishers    
        position_pub;
        velocity_pub;
        effort_pub;
        
        % data
        transforms = cell(1,6);
        q
        qd
        tau
        collided
 
        q_opponent      
        qd_opponent
        model_data 
        
        % static
        JOINT_NAMES = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"];

    end
    
    
    
    
    methods
        
        function obj = ArmController(color)

            if nargin == 0
                namespace = '';
            else
                namespace=strcat('/',color);
            end
            
            try
                rosshutdown
                rosinit
            catch ME
                % that's ok
            end
            
            obj.joint_sub = rossubscriber(strcat(namespace,'/arm_interface/state'),@obj.joint_cb);
            obj.collided_sub = rossubscriber(strcat(namespace,'/arm_interface/collided'),@obj.collided_cb);

            obj.pose_sub = {}; 

            for i = 1:6
                name = strcat(namespace,'/joint_poses/')+obj.JOINT_NAMES(i);
                obj.pose_sub{i} = rossubscriber(name,{@obj.transform_cb, i});
            end 

            obj.model_sub = rossubscriber('/gazebo/model_states',@obj.model_cb);

            
            obj.position_pub = rospublisher(strcat(namespace,'/arm_interface/position'), 'sensor_msgs/JointState');  
            obj.velocity_pub = rospublisher(strcat(namespace,'/arm_interface/velocity'), 'sensor_msgs/JointState');  
            obj.effort_pub = rospublisher(strcat(namespace,'/arm_interface/effort'), 'sensor_msgs/JointState');  


            % OPPONENT SENSING
    
            % find all namespaced lynx arms in sim with any namespace
            topiclist = rostopic("list");
            is_state_topic = cellfun(@(s) contains(s,'/arm_interface/state'), topiclist); arms = topiclist(is_state_topic);
            % figure out which one is the opponent (chooses the first
            % non-matching namespace)
            namespaces = cellfun(@(s) regexp(s,'\/(.*)\/arm_interface\/state','tokens'), arms, 'UniformOutput', false);
            if  numel([namespaces{:}]) > 0
                namespaces = cellfun(@(C) C{1,1,1},namespaces);
                opponent = find(cellfun(@(name) ~strcmp(name,color), namespaces));
                % subscribe to opponent's state
                if ~isempty(opponent)
                    obj.opponent_sub = rossubscriber(arms{opponent},@obj.opponent_cb);
                end
            end
            % if no opponent, no subscriber is created 
            
        end
        
        function stop(obj)
            rosshutdown;
        end

        function set_pos(obj, q)
            msg = rosmessage('sensor_msgs/JointState');
            msg.Position = q;
            send(obj.position_pub,msg);
        end

        function set_vel(obj, qd)
            msg = rosmessage('sensor_msgs/JointState');
            msg.Velocity = qd;
            send(obj.velocity_pub,msg);
        end

        function set_tau(obj, tau)
            msg = rosmessage('sensor_msgs/JointState');
            msg.Effort = tau;
            send(obj.effort_pub,msg);
        end

        function answer = is_collided(obj)
           answer = obj.collided;
        end

        function tf = get_poses(obj)
           tf = obj.transforms; 
        end

        function [q,qd] = get_state(obj)
           q = obj.q; 
           qd = obj.qd; 
        end

        function [q,qd] = get_opponent_state(obj)
           q = obj.q_opponent; 
           qd = obj.qd_opponent; 
        end

        function [name,pose,twist] = get_object_state(obj)
            
            if isempty(obj.model_data)
                name = {};
                pose = {};
                twist = {};
                return
            end
            
            is_cube = cellfun(@(s) contains(s,'cube'), obj.model_data.Name);
            cubes = find(is_cube);
            N = numel(cubes);
            name = obj.model_data.Name(is_cube);
            pose = cell(N,1);
            twist = cell(N,1);
            for i=1:N
                twist{i} = [
                    obj.model_data.Twist(cubes(i)).Linear.X
                    obj.model_data.Twist(cubes(i)).Linear.Y
                    obj.model_data.Twist(cubes(i)).Linear.Z
                    obj.model_data.Twist(cubes(i)).Angular.X
                    obj.model_data.Twist(cubes(i)).Angular.Y
                    obj.model_data.Twist(cubes(i)).Angular.Z
                ];
                        
                p = 1000*[
                    obj.model_data.Pose(cubes(i)).Position.X
                    obj.model_data.Pose(cubes(i)).Position.Y
                    obj.model_data.Pose(cubes(i)).Position.Z
                ];

                quat = quaternion(obj.model_data.Pose(cubes(i)).Orientation.W,...
                                  obj.model_data.Pose(cubes(i)).Orientation.X,...
                                  obj.model_data.Pose(cubes(i)).Orientation.Y,...
                                  obj.model_data.Pose(cubes(i)).Orientation.Z);

                R = rotmat(quat,'point');

                pose{i} = [ R p; 0 0 0 1];

            end
        end
        
    end
    
    methods (Access = private)
        

        function joint_cb(source, ~, data) 
            source.q = data.Position.';
            source.qd = data.Velocity.';
            source.tau = data.Effort.';
        end

        function opponent_cb(source, ~, data) 
            source.q_opponent = data.Position.';
            source.qd_opponent = data.Velocity.';
        end

        function transform_cb(source, ~, trans, i)
                
            p = 1000*[
                trans.Transform.Translation.X
                trans.Transform.Translation.Y
                trans.Transform.Translation.Z
            ];

            quat = quaternion(trans.Transform.Rotation.W,...
                              trans.Transform.Rotation.X,...
                              trans.Transform.Rotation.Y,...
                              trans.Transform.Rotation.Z);

            R = rotmat(quat,'point');

            Ti = eye(4,4);

            Ti(1:3,1:3) = R;
            Ti(1:3,4) = p;
            source.transforms{i} = Ti;

  
        end
          
        function collided_cb(source, ~, data) 
            source.collided = data.Data;
        end

        function model_cb(source, ~, data) 
            source.model_data = data;
        end

        
    end
    
end