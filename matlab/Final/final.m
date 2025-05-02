function final(color)

    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)

    % interact with simulator, such as...

    % get state of your robot
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
    [name,pose,twist] = lynx.get_object_state()

    % get state of your opponent's robot
    [q,qd]  = lynx.get_opponent_state()

end