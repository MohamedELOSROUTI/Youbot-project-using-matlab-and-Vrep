function drop(centerBasket, vrep, id)    
    timestep = .05;
    centerBasket = centerBasket(1:2);
    run('C:\Users\elosr\IntelligentRoboticsProject\trs\matlab\startup_robot.m')
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    addpath('C:\Users\elosr\IntelligentRoboticsProject\trs\youbot');
% % %     vrep = remApi('remoteApi');
% % %     vrep.simxFinish(-1);
% % %     id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
% % %     
    % If you get an error like: 
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

% % %     if id < 0
% % %         disp('Failed connecting to remote API server. Exiting.');
% % %         vrep.delete();
% % %         return;
% % %     end
% % %     fprintf('Connection %d to remote API server open.\n', id);
    
    % Make sure we close the connection whenever the script is interrupted.
% % %     cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
    
    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
% % %     vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    % The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the 
    % file focused/youbot_arm.m). 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);
    
    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    %startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    droppingJoints = [-pi/2, pi, 0.2, 0, 0];

    %% Preset values for the demo. 
% % %     disp('Starting robot');
    
    % Define the preset pickup pose for this demo. 
%     for i = 1:5
%         res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
%         vrchk(vrep, res, true);
%     end
    
% % %     res = vrep.simxPauseCommunication(id, false); 
% % %     vrchk(vrep, res);

    % Make sure everything is settled before we start. 
    pause(2);
    % First make sure that the youbot is correctly placed around the table.
    % The -y axis of the youbot should face table !
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res);
    centerBasketToYoubot = ...
        youbotPos(1:2)-centerBasket;
    tangentTableVec = [cosd(90), -sind(90);...
                   sind(90), cosd(90)]*...
                   centerBasketToYoubot';
    tangentTableVec = tangentTableVec/...
        norm(tangentTableVec);
    beta = wrapToPi(angle(tangentTableVec(1)+...
        complex(0,1)*tangentTableVec(2))-pi/2); % between [0,pi]
    
    % YoubotEuler(3) should be close to beta to start grasping
    % check it
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res);
    if (abs(youbotEuler(3) - beta) > 0.01)
        disp('Adjusting orientation of youbot');
        while (abs(youbotEuler(3) - beta) > 0.01) % 11 degree
            start_loop3 = tic;
            rotateRightVel = angdiff(beta, youbotEuler(3))/5;
            h = youbot_drive(vrep, h, 0, 0, rotateRightVel);
            [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res);
            ellapsed = toc(start_loop3);
            remaining = timestep - ellapsed;
            % time control
            if remaining > 0
                pause(min(remaining, .01));
            end
        end
    end

    
    h = youbot_drive(vrep, h, 0, 0, 0);
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    while (norm(centerBasketToYoubot) > 0.63)
        start_loop4 = tic;
        centerBasketToYoubot = ...
        youbotPos(1:2)-centerBasket;
        h = youbot_drive(vrep, h, 0, -0.05, 0);
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res);
        ellapsed = toc(start_loop4);
        remaining = timestep - ellapsed;
        if remaining > 0
                pause(min(remaining, .01));
        end
    end
    pause(0.01);
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);
    h = youbot_drive(vrep, h, 0, 0, 0);
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), droppingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    pause(3)
    % Open the gripper now
    res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep,res);
end
