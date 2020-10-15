function con = rosAddCommand(con, paths)
% rosAddCommand : to add command to robot arm
%
% INPUTS: 
%   con: the object includes methods and properties to handle the robot arm
%   paths: n*6 Joint variables for the six DOF Lynx arm
%
% OUTPUTS: 
%   con: the object includes methods and properties to handle the robot arm
%(because in the process, the object properties will change, so it needs to output)



    disp(con.cur_state);
    con = add_command(con,paths);
    pause(1);
    disp("[INFO]: Wait for new command...");

end