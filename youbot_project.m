function youbot_project()
    clc, clearvars, close all

    run('..\..\matlab\startup_robot.m')
    
    
    %% Parameters
    use_getPosition = true;
    plot_data = true;
    
    nb_iter = 0;
    nb_iter_per_plot = 10;
    
    %%%%% Parameters not used yet (not necessarily necessary)
    front_angle = 15; %°
    
    remap_distance = 1.5; %m
    remap = false;
    %%%%%

    targets_q = Queue();
    
    mapping_fig = figure('Name', 'Mapping figure');
    

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
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    % Move straight ahead. 
    robotVel = [0,0]; % Go sideways. 
    rotateRightVel = 0; % Rotate. 
    prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed). 
    prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed).
    
    
    % Youbot's map
    % Replaced custom OccupancyMap with matlab OccupancyGrid
    % Removed Dstar
    map = robotics.BinaryOccupancyGrid(50, 50, 4);
    % Inflated copy of map
    imap = map;
    
    
    controller = robotics.PurePursuit;
    path = [-2 -5.25; -2 0; -4 4; -6 3];
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 1;
    controller.MaxAngularVelocity = 1;
    controller.LookaheadDistance = 1;
    for i = 2:size(path, 1)
        targets_q.push(path(i,:));
    end
    cur_target = targets_q.front();
    

    %% Position setup
    % Youbot initial position
    [res, originPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevPosition = originPos(1:2);
    [res, originEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevOrientation = originEuler(3);
    

    % Voir comment l'enlever
    [X, Y] = meshgrid(-5:0.25:5, -5.5:0.25:2.5);
    
    
    % Finite State Machine first state
    fsm = 'computePath';
    
    %% Start
    disp('Enter loop')
    while true
        start_loop = tic;
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
        
        
        % Get the position and the orientation of the robot.
        % Mean=2.7E-4s  /  Min=1.8E-4s  /  Max=9.5ms
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        
        % Youbot rotation matrix (to put local coordinates in global axes)
        rotationMatrix = ...
            [cos(youbotEuler(3)) -sin(youbotEuler(3)) 0;...
             sin(youbotEuler(3)) cos(youbotEuler(3))  0;...
             0                   0                    1];
        
        
        %% Mapping
        % Put condition in front of block if not mapping
       
        % Get sensor infos
        % Mean=23ms  /  Min=16ms  /  Max=114ms
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer); % Mean=1ms
        in = inpolygon(X, Y,...
            [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
            [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]); % Assez lent, voir pour l'enlever ou accélérer

        % Rotated points (to avoid using sine and cosine in code)
        r_pts = rotationMatrix*pts;


        % Retrieve location information
        % Mean=5.4E-4s  /  Min=3.2E-4s  /  Max=10.5ms
        % See if x_pts and y_pts are needed
        x_pts = youbotPos(1) - originPos(1) + X(in)*cos(youbotEuler(3)) - Y(in)*sin(youbotEuler(3));
        x_contact = youbotPos(1) - originPos(1) + r_pts(1, contacts);

        y_pts = youbotPos(2) - originPos(2) + Y(in)*cos(youbotEuler(3)) + X(in)*sin(youbotEuler(3));
        y_contact = youbotPos(2) - originPos(2) + r_pts(2, contacts);

        
%         setOccupancy(map2, [25+x_pts 25+y_pts], 0.001); %Sauf si
%         correctif possible, ne fonctionne pas correctement : remplace les
%         obstacles par des non obstacles
        setOccupancy(map, [25+x_contact' 25+y_contact'], 1);
        
        
        %% Finite State Machine
        if strcmp(fsm, 'computePath')
            %% Compute path
            
            
            
        end
                
        
        %% Too close from obstacle condition
        % Check in front of the robot
%         in_front = abs(pts(1,:)./pts(2,:)) < tan(front_angle*pi/180);
%         d = sqrt(r_pts(1,contacts & in_front).^2 + r_pts(2,contacts & in_front).^2);
%         if min(d) < remap_distance
%             fsm = 'computePath';
%         end
        
        
        %% Robot driving
        if norm(youbotPos(1:2) - cur_target) < 0.4
            if cur_target == path(end,:)
                robotVel = [0 0];
                rotateRightVel = 0;
            else
                targets_q.pop_front();
                cur_target = targets_q.front();
            end
        else
            [v, omega] = controller([youbotPos(1:2) transformAngleRange(youbotEuler(3), pi/2, [-pi pi])]);
            robotVel = [v 0];
            rotateRightVel = omega;
        end
        % Get position in youbot coordinate system
        % Use tranpose of rotationMatrix because multiplating with a line
        % matrix instead of a column matrix
        relPos = ([cur_target 0] - youbotPos)/rotationMatrix';
        % Get angle between front of youbot and target
        agl = atan(relPos(1)/relPos(2));
        % Compose speed
        targetRMatrix =...
            [cos(agl)  sin(agl);...
             -sin(agl) cos(agl)];
        robotVel = robotVel * targetRMatrix;
        
        h = youbot_drive(vrep, h, -robotVel(1), -robotVel(2), -rotateRightVel);
        
        
        % Previous position and orientation
        % Check if necessary (because no longer use prev pos and ori)
        prevPosition = youbotPos(1:2);
        prevOrientation = youbotEuler(3);
        
        
        %% Plotting
        % Show map (if needed: see 'plot_data' )
        % Mean=19.4ms  /  Min=10.5ms  /  Max=440ms
        if plot_data && mod(nb_iter, nb_iter_per_plot) == 0
            figure(mapping_fig);
            
            show(map);
            hold on;
            plot(path(:,1)+25-originPos(1), path(:,2)+25-originPos(2),'k--d')
            scatter(25+youbotPos(1)-originPos(1), 25+youbotPos(2)-originPos(2), '*', 'r')
            hold off;
        end
        
        % Count number of times while is executed to save execution time on
        % mapping
        nb_iter = nb_iter + 1;
        
        
        %% Calculation time control
        ellapsed = toc(start_loop);
        remaining = timestep - ellapsed;
        if remaining > 0
            pause(min(remaining, .01));
        end
    end
end