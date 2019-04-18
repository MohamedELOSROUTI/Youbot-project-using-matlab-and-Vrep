function youbot_project()
    clc, clearvars, close all

    run('..\..\matlab\startup_robot.m')
    
    
    %% Parameters
    use_getPosition = true;
    plot_data = true;
    mapping = true;
    
    nb_iter = 0;
    nb_iter_per_plot = 10;
    
    %%%%% Parameters not used yet (not necessarily necessary)
    front_angle = 15*pi/180; %Â°
    
    remap_distance = 1.5; %m
    remap = false;
    %%%%%
    
    robot_radius = 0.4;

    targets_q = Queue();
    path = [];
    repath = true;
    
    mapping_fig = figure('Name', 'Mapping figure');
    
    
    e = zeros([1 1000]);
    

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
    map = robotics.OccupancyGrid(50, 50, 5);
    [x, y] = meshgrid(linspace(-0.5, 0.5, 20));
    R = hypot(x, y);
    setOccupancy(map, 25+[x(R < robot_radius) y(R < robot_radius)], 0);
    % Inflated copy of map
    imap = copy(map);
    
    
    controller = robotics.PurePursuit;
    controller.DesiredLinearVelocity = 1;
    controller.MaxAngularVelocity = 1;
    controller.LookaheadDistance = 1;
    
    prm = robotics.PRM;
    prm.Map = imap;
    prm.NumNodes = 20;
    prm.ConnectionDistance = 3;
    

    %% Position setup
    % Youbot initial position
    [res, originPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevPosition = originPos(1:2);
    [res, originEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevOrientation = originEuler(3);
    
    cur_target = originPos(1:2);
    

    % Voir comment l'enlever
    [X, Y] = meshgrid(-5:0.2:5, -5.5:0.2:2.5);
    
    
    % Finite State Machine first state
    fsm = 'firstLook';
    
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
        if mapping
            
            % Get sensor infos
            % Mean=23ms  /  Min=16ms  /  Max=114ms
            % Look at insertRay or raycast to replace inpolygon (bc quite
            % slow)
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer); % Mean=1ms
            in = inpolygon(X, Y,...
                [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
                [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]); % Assez lent, voir pour l'enlever ou accÃ©lÃ©rer

            % Rotated points (to avoid using sine and cosine in code)
            r_pts = rotationMatrix*pts;


            % Retrieve location information
            % Mean=5.4E-4s  /  Min=3.2E-4s  /  Max=10.5ms
            % See if x_pts and y_pts are needed
            x_pts = youbotPos(1) - originPos(1) + X(in)*cos(youbotEuler(3)) - Y(in)*sin(youbotEuler(3));
            x_contact = youbotPos(1) - originPos(1) + r_pts(1, contacts);

            y_pts = youbotPos(2) - originPos(2) + Y(in)*cos(youbotEuler(3)) + X(in)*sin(youbotEuler(3));
            y_contact = youbotPos(2) - originPos(2) + r_pts(2, contacts);
            
            
            % Update occupancy grid cases probabilities
            updateOccupancy(map, [25+x_pts 25+y_pts], 0.4);
            updateOccupancy(map, [25+x_contact' 25+y_contact'], 0.7);
            
        end
        
        
        %% Finite State Machine
        if strcmp(fsm, 'firstLook')
            %% First look
            % Enforce free and occupied probabilities in grid around youbot
            % original pose
            if nb_iter == 10
                fsm = 'computePath';
                disp('Compute path');
            end
        elseif strcmp(fsm, 'computePath')
            %% Compute path
            
            if repath
                % Inflate map
                imap = copy(map);
                inflate(imap, robot_radius);
                prm.Map = imap;
                
                free_cells = [];
                front_angle = 5*pi/180;
                
                startl = 25 + youbotPos(1:2) - originPos(1:2);
                % Ajouter le choix de point aléatoire ici
%                 endl = 25 + youbotPos(1:2) - originPos(1:2) + [0 -4];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Première idée pour choisir un point aléatoire
                %  Conditions :
                %   * Rayon compris entre 3 et 4 m
                %   * Dans un cone de 15° (vérifier si ça fonctionne, on
                %   sait jamais)
                %
                %  -> Regarder les cases libres dans cette zone
                %  -> Ajouter une boucle si on trouve pas de case
                %  respectant ces conditions
                %  ->
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                [x, y] = meshgrid(-4:0.2:4, 4:-0.2:-4);
                R = hypot(x, y);
                
                pos = ([x(1,:)' y(:,1)] - youbotPos(1:2))/rotationMatrix(1:2,1:2)';
                T = transformAngleRange( atan2(x, y) + youbotEuler(3), pi, [-pi pi] );
                
                while isempty(free_cells)
                    front_angle = front_angle + 10*pi/180;
                    
                    cells = 25+[x(R < 4 & R > 3 & abs(T) < 2*front_angle) y(R < 4 & R > 3 & abs(T) < 2*front_angle)]...
                        + youbotPos(1:2) - originPos(1:2);
                    
                    ij = unique(world2grid(imap, cells), 'rows');
                    occ = checkOccupancy(imap, ij, 'grid');
                    ij_occ = [ij occ];
                    
                    free_cells = ij_occ( ij_occ(:,3) == 0 ,1:2);
                end
                endl = grid2world( imap, free_cells(randi(length(free_cells)),:) );
                
                
                % While cannot find path, add nodes
                while isempty(path)
                    prm.NumNodes = prm.NumNodes + 10;
                    prm.update();
                    
                    path = findpath(prm, double(startl), double(endl));
                end
                
                % Pass path found to controller and targets_q
                path = path - 25 + originPos(1:2);
                controller.Waypoints = path;
                for i=2:size(path, 1)
                    targets_q.push(path(i,:));
                end
                cur_target = targets_q.front();
                
                % No need to repath yet
                repath = false;
            end
            
        end
                
        
        %% Too close from obstacle condition
        % May be useless now because point choice is only in known area
        % Check in front of the robot
%         in_front = abs(pts(1,:)./pts(2,:)) < tan(front_angle*pi/180);
%         d = sqrt(r_pts(1,contacts & in_front).^2 + r_pts(2,contacts & in_front).^2);
%         if min(d) < remap_distance
%             fsm = 'computePath';
%         end
        
        
        %% Robot driving
        % If encounter final goal
        if ~isempty(path)
            % In case we are on the goal
            if norm(youbotPos(1:2) - path(end,:)) < 0.4
                targets_q.clear();
                cur_target = path(end,:);
            end
        end
        % If close to intermediate path target
        if norm(youbotPos(1:2) - cur_target) < 0.4
            if targets_q.NumElements < 2
                targets_q.clear();
                
                robotVel = [0 0];
                rotateRightVel = 0;
                
                repath = true;
                path = [];
            else
                targets_q.pop_front();
                cur_target = targets_q.front();
            end
        else
            [v, omega] = controller([youbotPos(1:2) transformAngleRange(youbotEuler(3), pi/2, [-pi pi])]);
            robotVel = [v 0];
            rotateRightVel = omega;
            
            % Get position in youbot coordinate system
            % Use tranpose of rotationMatrix because multiplating with a line
            % matrix instead of a column matrix
            relPos = ([cur_target 0] - youbotPos)/rotationMatrix';
            % Get angle between front of youbot and target
            % A corriger
            agl = transformAngleRange(atan(relPos(1)/relPos(2)) + youbotEuler(3), pi, [-pi pi]);
            % Compose speed
            targetRMatrix =...
                [cos(agl)  sin(agl);...
                 -sin(agl) cos(agl)];
            robotVel = robotVel * targetRMatrix;
        end
        
        h = youbot_drive(vrep, h, -abs(robotVel(1)), -robotVel(2)/2, -rotateRightVel);
        
        
        % Previous position and orientation
        % Check if necessary (because no longer use prev pos and ori)
        prevPosition = youbotPos(1:2);
        prevOrientation = youbotEuler(3);
        
        
        %% Plotting
        % Show map (if needed: see 'plot_data' )
        % Mean=19.4ms  /  Min=10.5ms  /  Max=440ms
        if plot_data && mod(nb_iter, nb_iter_per_plot) == 0
            figure(mapping_fig);
            
            subplot(2,1,1);
            show(map);
            hold on;
            if ~isempty(path)
                plot(path(:,1)+25-originPos(1), path(:,2)+25-originPos(2),'k--d')
            end
            scatter(25+youbotPos(1)-originPos(1), 25+youbotPos(2)-originPos(2), '*', 'r')
            hold off;
            
            subplot(2,1,2);
            if strcmp(fsm, 'computePath')
                show(prm)
            end
        end
        
        % Count number of times while is executed to save execution time on
        % mapping
        nb_iter = nb_iter + 1;
        
        
        %% Calculation time control
        ellapsed = toc(start_loop);
        e(find(e==0, 1, 'first')) = ellapsed;
        remaining = timestep - ellapsed;
        if remaining > 0
            pause(min(remaining, .01));
        end
%         mean(e(e~=0));
    end
end