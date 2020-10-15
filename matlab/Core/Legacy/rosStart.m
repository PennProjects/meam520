function con = rosStart(enableGripper)
% rosStart: to start ros node and create the object
%
% INPUTS: 
%   enableGripper: set "true" if you want to use gripper mode, "false" change to lidar mode
%
% OUTPUTS: 
%   con: the object includes methods and properties to handle the robot arm
 
    % step one:  make sure no running ros and clean the workspace
    rosshutdown;
    
    % step two: inital ros node 
    rosinit;

    % step three: create the object con
    con = ArmController(enableGripper);
    
end