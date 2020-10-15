classdef ArmController < handle
    properties (Access = private)    
                
        % subscribers
        joint_sub  
        pose_sub
        collided_sub  
        
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
              
        % static
        JOINT_NAMES = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"];

    end
    
    methods
        
        function obj = ArmController()

            rosshutdown;
            rosinit;

            obj.joint_sub = rossubscriber('/arm_interface/state',@obj.joint_cb);
            obj.collided_sub = rossubscriber('/arm_interface/collided',@obj.collided_cb);

            obj.pose_sub = {}; 

            for i = 1:6
                name = '/joint_poses/'+obj.JOINT_NAMES(i);
                obj.pose_sub{i} = rossubscriber(name,{@obj.transform_cb, i});
            end 

            obj.position_pub = rospublisher("/arm_interface/position", 'sensor_msgs/JointState');  
            obj.velocity_pub = rospublisher("/arm_interface/velocity", 'sensor_msgs/JointState');  
            obj.effort_pub = rospublisher("/arm_interface/effort", 'sensor_msgs/JointState');  
            
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

    end
    
    methods (Access = private)

        function joint_cb(source, ~, data) 
            source.q = data.Position.';
            source.qd = data.Velocity.';
            source.tau = data.Effort.';
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

    end
    
    
end