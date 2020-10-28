clear

addpath('../Core') % references ROS interface / arm controller you'll need for every lab

lynx = ArmController();

pause(1) % wait for setup

clc

disp("Returning to start position:")
lynx.set_pos(zeros(1,6))
pause(5)

dq = [.5,-0.5,-0.5, -0.1,1.2,0];

disp("Target Velocity:")
disp(dq)

lynx.set_vel(dq)

pause(2)

lynx.set_vel(zeros(1,6))

[pos, vel] = lynx.get_state();

disp("Result:")
disp(pos)

lynx.stop() % Shut down ROS interface