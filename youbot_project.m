function youbot_project()
    clc, clear all, close all

    run('../../matlab/startup_robot.m')
    
    
    %% Parameters
    use_getPosition = true;
    
    
    %% Initiate the connection to the simulator. 
    disp('Program started');
    
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    
    % Make sure to close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
    
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
    
    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    
    pause(.2);
    
    
    %% Youbot constants
    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    
    
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 
    prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed). 
    prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed).
    
    
    % Youbot's map
    map = OccupancyMap([201 201], 0.25);
    center = round( (size(map.Map) + 1) /2 );
    
    
    % Youbot initial position
    [res, originPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevPosition = originPos(1);
    [res, originEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevOrientation = originEuler(3);
    
    
    % Voir comment l'enlever
    [X, Y] = meshgrid(-5:map.MapRes:5, -5.5:map.MapRes:2.5);
    
    
    % Finite State Machine first state
    fsm = 'map';
    
    %% Start
    disp('Enter loop')
    while true
        tic
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
        
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        
        %% Finite State Machine
        if strcmp(fsm, 'map')
            % Get sensor infos
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
            in = inpolygon(X, Y,...
                [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
                [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);
            
            % Retrieve location information
            x_pts = youbotPos(1) - originPos(1) + X(in)*cos(youbotEuler(3)) - Y(in)*sin(youbotEuler(3));
            x_contact = youbotPos(1) - originPos(1) + pts(1, contacts)*cos(youbotEuler(3)) - pts(2, contacts)*sin(youbotEuler(3));

            y_pts = youbotPos(2) - originPos(2) + Y(in)*cos(youbotEuler(3)) + X(in)*sin(youbotEuler(3));
            y_contact = youbotPos(2) - originPos(2) + pts(2, contacts)*cos(youbotEuler(3)) + pts(1, contacts)*sin(youbotEuler(3));

            map.addPoints(-round(y_pts/map.MapRes) + center(1), round(x_pts/map.MapRes) + center(2), map.Free);
            map.addPoints(-round(y_contact/map.MapRes) + center(1), round(x_contact/map.MapRes) + center(2), map.Wall);
        end
        
        
        %% Youbot drive system
        % Compute angle velocity
        rotateRightVel = 0 - youbotEuler(3);
        if (abs(angdiff(0, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
            rotateRightVel = 0;
        end
        % Compute forward velocity
        forwBackVel = youbotPos(1) + 5;
        if (youbotPos(1) + 5 < .001) && (abs(youbotPos(1) - prevPosition) < .001)
            forwBackVel = 0;
        end
        
        % Previous position and orientation
        prevPosition = youbotPos(1);
        prevOrientation = youbotEuler(3);
        
        % Set youbot velocities
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
        
        
        % Show map
        map.show()
        hold on
        scatter(youbotPos(1) - originPos(1) + center(1)*map.MapRes, -(youbotPos(2) - originPos(2)) + center(2)*map.MapRes, '*', 'r')
        
        
        %% Calculation time control
        ellapsed = toc;
        remaining = timestep - ellapsed;
        if remaining > 0
            pause(min(remaining, .01));
        end
    end
end