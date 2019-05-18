function youbot_project()
    clc, clearvars, close all

    run('..\..\matlab\startup_robot.m')
    
    
    %% Parameters
    use_getPosition = true;
    plot_data = true;
    mapping = true;
    
    nb_iter = 0;
    nb_iter_per_plot = 5;
    
    front_angle = 5*pi/180; %degrees
    robot_radius = 0.3;
    prevPMaxRange = 5;
    passedPoints = [];
    closePP_map = [];
    intsecPts = [];
    
    occpct = zeros(1,5);
    occpct_i = 0;

    targets_q = Queue();
    path = [];
    repath = true;
    
    
    mapping_fig = figure('Name', 'Mapping figure');
    set(gcf, 'position', [10 10 980 520]);
    

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
    mapSize = [50 50];
    map = robotics.OccupancyGrid(mapSize(1), mapSize(2), 16);
    [x, y] = meshgrid(linspace(-1, 1, 41));
    R = hypot(x, y);
    setOccupancy(map, 25+[x(R <= 2.5*robot_radius) y(R <= 2.5*robot_radius)], 0.1);
    % Inflated copy of map
    imap = copy(map);
    
    
    controller = robotics.PurePursuit;
    controller.DesiredLinearVelocity = 1;
    controller.MaxAngularVelocity = 1;
    controller.LookaheadDistance = 1;
    
    prm = robotics.PRM;
    prm.Map = imap;
    prm.NumNodes = 70;
    prm.ConnectionDistance = 5;
    

    %% Position setup
    % Youbot initial position
    [res, originPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevPosition = originPos(1:2);
    [res, originEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    prevOrientation = originEuler(3);
    
    
    cur_target = originPos(1:2);
    
    
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
        
        % Youbot position in map frame
        youbotPos_map = youbotPos - originPos + [mapSize/2 0];
        
        
        %% Mapping
        if mapping
            
            % Get sensor infos
            % Mean=23ms  /  Min=16ms  /  Max=114ms
            % Look at insertRay or raycast to replace inpolygon (bc quite
            % slow)
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);

            % Rotated points (to avoid using sine and cosine in code)
            r_pts = rotationMatrix*pts;
            
            % Update occupancy grid cases probabilities
            insertRay(map, youbotPos_map(1:2), downsample(r_pts(1:2,:)', 5) + youbotPos_map(1:2), [0.2 0.5]);
            updateOccupancy(map, r_pts(1:2,contacts)' + youbotPos_map(1:2), 1);
            
        end
        
        
        %% Finite State Machine
        if strcmp(fsm, 'firstLook')
            %% First look
            % Enforce free and occupied probabilities in grid around youbot
            % original pose
            if nb_iter == 2
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
                
                startl = youbotPos_map(1:2);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Premi�re id�e pour choisir un point al�atoire
                %  Conditions :
                %   * Rayon compris entre 3 et 4 m
                %   * Dans un cone de 15� (v�rifier si �a fonctionne, on
                %   sait jamais)
                %
                %  -> Regarder les cases libres dans cette zone
                %  -> Ajouter une boucle si on trouve pas de case
                %  respectant ces conditions
                %  ->
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                norms = vectnorm(youbotPos_map(1:2) - passedPoints, 2);
                closePP = youbotPos_map(1:2) - passedPoints( 0.1 < norms & norms < prevPMaxRange ,:);
                closePP_map = youbotPos_map(1:2) - closePP;
                if ~isempty(closePP)
                    agls = atan2(closePP(:,1), -closePP(:,2));
                    norms = norms(0.1 < norms & norms < prevPMaxRange);

                    wgt = (prevPMaxRange - norms)/prevPMaxRange;
                    
                    % Get if prev point is behind a wall
                    % V�rifier si �a fonctionne
                    intsecPts = rayIntersection(map, [youbotPos_map(1:2) 0], agls', prevPMaxRange+0.1, 0.8);
                    wgt(~isnan(intsecPts(:,1))) = wgt(~isnan(intsecPts(:,1)))/2;
                    intsecPts(isnan(intsecPts)) = closePP(isnan(intsecPts));
                    
                    sumprod_agls = transformAngleRange( meanAngle(agls, wgt), 0, [-pi pi] );
                else 
                    sumprod_agls = youbotEuler(3);
                end
                sumprod_agls*180/pi
                
                
                [x, y] = meshgrid(-4:0.2:4, 4:-0.2:-4);
                R = hypot(x, y);
                T = transformAngleRange( atan2(x, -y), -sumprod_agls, [-pi pi] );
                
                while isempty(free_cells)
                    front_angle = front_angle + 5*pi/180;
                    
                    cells = [x(R < 4 & R > 3 & abs(T) < 2*front_angle) ...
                             y(R < 4 & R > 3 & abs(T) < 2*front_angle)]...
                        + youbotPos_map(1:2);
                    
                    ij = unique(world2grid(imap, cells), 'rows');
                    occ = checkOccupancy(imap, ij, 'grid');
                    ij_occ = [ij occ];
                    
                    free_cells = ij_occ( ij_occ(:,3) == 0 ,1:2);
                end
                endl = grid2world( imap, free_cells(randi(size(free_cells, 1)),:) );
                
                % While cannot find path, add nodes
                initialNumNodes = prm.NumNodes;
                while isempty(path)
                    prm.NumNodes = prm.NumNodes + 5;
                    prm.update();
                    
                    path = findpath(prm, double(startl), double(endl));
                    if prm.NumNodes > initialNumNodes+50
                        endl = grid2world( imap, free_cells(randi(size(free_cells, 1)),:) );
                        prm.NumNodes = initialNumNodes;
                    end
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
        elseif strcmp(fsm, 'locateTablesBaskets')
            map_ = occupancyMatrix(map, 'ternary');
            imap_ = occupancyMatrix(imap, 'ternary');
            figure(2)
            
            [centers, radii, metric] = imfindcircles(imap_, [10 13], 'ObjectPolarity', 'bright', 'Sensitivity', 1);
            %% Choose the most significant circles metric lim = 0.05 => find the basket - tables
            % we have to find a way to distinguish them now
            centersStrong = centers(metric>0.05,:);
            radiiStrong = radii(metric>0.05);
            metricStrong = metric(metric>0.05);
            idx = sub2ind(size(imap_),centersStrong(:,1),centersStrong(:,2));
            idx = int64(idx);
            % give a particular number (10-16) to each tables and baskets
            % in order to localize them easily in the future
            for i = 10:16
                map_(idx(i-9)) = i;
            end
            imshow(map_)
            viscircles(centersStrong, radiiStrong, 'EdgeColor', 'b');
        
        elseif strcmp(fsm, 'end')
            
        end
        
        
        %% Robot driving
        % If encounter final goal
        if ~isempty(path)
            % In case we are on the goal
            if norm(youbotPos(1:2) - path(end,:)) < 0.2
                targets_q.clear();
                cur_target = path(end,:);
            end
        end
        % If close to intermediate path target
        if norm(youbotPos(1:2) - cur_target) < 0.2
            if targets_q.NumElements < 2
                targets_q.clear();
                
                robotVel = [0 0];
                rotateRightVel = 0;
                
                repath = true;
                path = [];
            
                % Add passed point and check unknown map occupancy
                % If occupancy is mostly the same 5 times in a row, there's
                % no more to discover in the map => go next
                if strcmp(fsm, 'computePath')
                    passedPoints = [passedPoints ; youbotPos_map(1:2)];
                    
                    
                    mat = occupancyMatrix(map, 'ternary');
                    occpct(occpct_i+1) = (1 - length(find(mat == -1))/( size(mat,1) * size(mat,2) ))*100;
                    
                    if all( abs(occpct - mean(occpct)) < 0.01 ) && ~all(occpct == 0)
                        disp('End');
                        
                        repath = false;
                        mapping = false;
                        fsm = 'locateTablesBaskets';
                    end
                    
                    occpct_i = mod(occpct_i+1, size(occpct,2));
                end
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
            relPos = -([cur_target 0] - youbotPos)/rotationMatrix';
            % Get angle between front of youbot and target
            % A corriger
            agl = atan2(-relPos(1), relPos(2));
            % Compose speed
            targetRMatrix =...
                [cos(agl)  sin(agl);...
                 -sin(agl) cos(agl)];
            robotVel = robotVel * targetRMatrix;
        end
        
        
        h = youbot_drive(vrep, h, -robotVel(1), robotVel(2), -rotateRightVel);
        
        
        % Previous position and orientation
        % Check if necessary (because no longer use prev pos and ori)
        prevPosition = youbotPos(1:2);
        prevOrientation = youbotEuler(3);
        
        
        %% Plotting
        % Show map (if needed: see 'plot_data' )
        % Mean=19.4ms  /  Min=10.5ms  /  Max=440ms
        if plot_data && mod(nb_iter, nb_iter_per_plot) == 0
            figure(mapping_fig);
            
            subplot(1,2,1);
            show(map);
            hold on;
            if ~isempty(path)
                plot(path(:,1)+mapSize(1)/2-originPos(1), path(:,2)+mapSize(2)/2-originPos(2),'k--d')
                scatter(closePP_map(:,1), closePP_map(:,2), 'b*');
                % Maybe add rayIntersect info and draw line between youbot
                % and points
            end
            scatter(25+youbotPos(1)-originPos(1), 25+youbotPos(2)-originPos(2), '*', 'r')
            hold off;
            %% if we are at state 'locateTablesBaskets' plot a cross on them
            if strcmp(fsm, 'locateTablesBaskets')
                scatter(centersStrong(:,1), centersStrong(:,2),'b*');
                fsm = 'end';
            end
            subplot(1,2,2);
            if strcmp(fsm, 'computePath') && ~isempty(path)
                show(prm)
            end
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