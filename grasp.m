function [stateObject] = grasp(centerTable,vrep,h,id,store)
    %% Function that takes as input the center of the table
    %  returns stateObject which takes 2 values :
    % 1. 'Piched' : The robot managed to find a cylinder and has taken it
    % from the table
    % 2. 'Unpicked' : means that no cylinder has been found
    % Arguments => store (boolean) tells if the object has to be placed on 
    % the gripper ('0') or stored on the youbot platform ('1').
    % Important : we assume that the youbot is facing the table in parallel
    % according to -y axis !
    timestep = .05;
    centerTable = centerTable(1:2);
    run('C:\Users\elosr\IntelligentRoboticsProject\trs\matlab\startup_robot.m')
% % %     disp('Program started');
    % Use the following line if you had to recompile remoteApi
%     vrep = remApi('remoteApi', 'extApi.h');
    addpath('C:\Users\elosr\IntelligentRoboticsProject\trs\youbot');
% % %     vrep = remApi('remoteApi');
% % %     vrep.simxFinish(-1);
% % %     id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
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
    [res, armRefToXyzSensor] = vrep.simxGetObjectPosition(id, h.xyzSensor, h.armRef, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    [res, youbotRefToXyzSensor] = vrep.simxGetObjectPosition(id, h.xyzSensor, h.ref, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    AngleSensor = pi/180*linspace(120,220,10);
    rotationx = [1,0,0,0;...
                0,cosd(90),-sind(90),0;...
                0,sind(90), cosd(90), 0;...
                0,0,0,1];
    rotationx2 = [1,0,0,0;...
                0,cosd(-90),-sind(-90),0;...
                0,sind(-90), cosd(-90), 0;...
                0,0,0,1];
    rotationz1 = [cosd(90),-sind(90),0,0;...
                sind(90),cosd(90),0,0;...
                0,0,1,0;...
                0,0,0,1];

    rotationz3 = [cosd(-90),-sind(-90),0,0;...
                sind(-90),cosd(-90),0,0;...
                0,0,1,0;...
                0,0,0,1];
    translation = [1,0,0,armRefToXyzSensor(1);...
                   0,1,0,armRefToXyzSensor(2);...
                   0,0,1,armRefToXyzSensor(3);...
                   0,0,0,1];
               
    transYoubotRefToXyzSensor = [1,0,0,youbotRefToXyzSensor(1);...
                                 0,1,0,youbotRefToXyzSensor(2);...
                                 0,0,1,youbotRefToXyzSensor(3);...
                                 0,0,0,1];
        
               
    Roty = [cosd(90),0,sind(90),0;...
            0,1,0,0;...
            -sind(90),0,cosd(90),0;...
            0,0,0,1];
        
    Roty2 = [cosd(-90),0,sind(-90),0;...
            0,1,0,0;...
            -sind(-90),0,cosd(-90),0;...
            0,0,0,1];
%     Rotz = [cosd(90),-sind(90),0,0;...
%                 sind(90),cosd(90),0,0;...
%                 0,0,1,0;...
%                 0,0,0,1];
%     trans = [1,0,0,youbotToArmRef(1);...
%              0,1,0,youbotToArmRef(2);...
%              0,0,1,youbotToArmRef(3);...
%              0,0,0,1];
         
    RotzRefToXYZ = [cosd(180),-sind(180),0,0;...
                sind(180),cosd(180),0,0;...
                0,0,1,0;...
                0,0,0,1];
    RotxRefToXYZ = [1,0,0,0;...
                0,cosd(-90),-sind(-90),0;...
                0,sind(-90), cosd(-90), 0;...
                0,0,0,1];
    
    %centerTable = [-3,-6];
    
    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);
    
    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    %placingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    % placingJoints1 = [0, 39 * pi / 180, 35 * pi / 180, 50 * pi / 180, 0];
    placingJoints2 = [0, 39 * pi / 180, 55 * pi / 180, 0 * pi / 180, 0]; % au lieu de 0 mettre 50

    %% Preset values for the demo. 
% % %     disp('Starting robot');
    
    % Define the preset pickup pose for this demo. 
%     for i = 1:5
%         res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), placingJoints(i), vrep.simx_opmode_oneshot);
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
    centerTableToYoubotPosVec = ...
        youbotPos(1:2)-centerTable;
    tangentTableVec = [cosd(90), -sind(90);...
                   sind(90), cosd(90)]*...
                   centerTableToYoubotPosVec';
    tangentTableVec = tangentTableVec/...
        norm(tangentTableVec);
    beta = wrapToPi(angle(tangentTableVec(1)+...
        complex(0,1)*tangentTableVec(2))-pi/2); % between [0,pi]
    
    % YoubotEuler(3) should be close to beta to start grasping
    % check it
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res);
    if (abs(youbotEuler(3) - beta) > 0.1)
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
    while (norm(centerTableToYoubotPosVec) > 0.66)
        start_loop4 = tic;
        centerTableToYoubotPosVec = ...
        youbotPos(1:2)-centerTable;
        h = youbot_drive(vrep, h, 0, -0.1, 0); % par defaut 0.05
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
 %%%%%%%%%%%%%%%%%%%%%%%%  main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i=1;
    while true
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
        takePicture(id,vrep,h,AngleSensor(i));
        pts = scanXYZ(id,vrep,h, AngleSensor(i), pi/15);
        rotationz2 = [cos(AngleSensor(i)),-sin(AngleSensor(i)),0,0;...
                sin(AngleSensor(i)),cos(AngleSensor(i)),0,0;...
                0,0,1,0;...
                0,0,0,1];
        rotationXSensor =[1,0,0,0;...
                          0,cos(AngleSensor(i)),-sin(AngleSensor(i)),0;...
                          0,sin(AngleSensor(i)), cos(AngleSensor(i)), 0;...
                          0,0,0,1];
                      
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res);
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res);
        transyoubotPos = [1,0,0,youbotPos(1);...
                          0,1,0,youbotPos(2);...
                          0,0,1,youbotPos(3);...
                          0,0,0,1];
        rotationzYoubotEuler = [cos(youbotEuler(3)),-sin(youbotEuler(3)),0,0;...
                                sin(youbotEuler(3)),cos(youbotEuler(3)),0,0;...
                                0,0,1,0;...
                                0,0,0,1];
        [center, Found] = detectCylinder(pts);
        % center is armRef axis
            NewCenter = [center(1);center(2);center(3);1];
            NewCenter = translation*rotationz2*rotationz1*rotationx*NewCenter;
            NewCenter = [NewCenter(1),NewCenter(2),NewCenter(3)];
        % center in youbot ref axis
            NewCenterYoubotRef = transYoubotRefToXyzSensor*...
                                 rotationXSensor*...
                                 rotationx2*...
                                 rotationz3*...
                                 [center(1);center(2);center(3);1];
                                 
        % center in absolute axis (-1)
            NewCenterAbsuluteAxis = transyoubotPos*...
                                    rotationzYoubotEuler*...
                                    rotationz3*...
                                    Roty2*...
                                    NewCenterYoubotRef;
           if Found == 0
               stateObject = 'not found';
           elseif norm(NewCenter) > 0.51
               stateObject = 'too far';
           else
               stateObject = 'reachable';
           end
           
        if strcmp(stateObject,'reachable')
            disp('Cylinder founded');

            % Activate IK 
        
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        
            % move the tip of the arm 
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, 0.5*[NewCenter(1),NewCenter(2),2*NewCenter(3)], vrep.simx_opmode_oneshot_wait);
            vrchk(vrep,res);
            pause(3);
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, NewCenter, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep,res);
            pause(3)
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, 1.05*NewCenter, vrep.simx_opmode_oneshot_wait);

            vrchk(vrep, res, true);
            disp('Moving gripper ...');
            pause(2);
            disp('Grasping ...');
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep,res);
            pause(5);
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep,res);
%             for i = 1:5
%                 res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), placingJoints1(i), vrep.simx_opmode_oneshot);
%                 vrchk(vrep, res, true);
%             end
%             pause(2);
            
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), placingJoints2(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            pause(2);
            if store
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
                vrchk(vrep,res);
            end
            stateObject = 'Picked';
            break;
        elseif strcmp(stateObject,'too far')
                disp('Cylinder too far, moving towards ...');
                
                % NewCenter in youbot axis 
            % NewCenterYoubot = trans*Roty*Rotz*[NewCenter,1]'; % Don't
            % provide good results, has to be corrected
            % NewCenterYoubot in ref axis
            [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res);
            [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res);

            [res,xyzSensor] = vrep.simxGetObjectPosition(id, h.xyzSensor, -1, vrep.simx_opmode_oneshot_wait);
            transRefxyzSensor = [1,0,0,xyzSensor(1);...
             0,1,0,xyzSensor(2);...
             0,0,1,xyzSensor(3);...
             0,0,0,1];
             vrchk(vrep, res);
             

            NewCenterRef =  transRefxyzSensor*...
                            rotationz2*...
                            RotxRefToXYZ*...
                            RotzRefToXYZ*...
                            [center(1);center(2);center(3);1];
            centerTableYoubot = youbotPos(1:2) - centerTable;
            N = 50; % 50
            aroundcenterTablePointsPos = centerTableYoubot(1:2)';
            aroundcenterTablePointsNeg = aroundcenterTablePointsPos;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            RotPos = [cosd(360/N), -sind(360/N);...
                   sind(360/N), cosd(360/N)];
            RotNeg = [cosd(-360/N), -sind(-360/N);...
                   sind(-360/N), cosd(-360/N)];
               for j = 2:N
                aroundcenterTablePointsPos(:,j) = RotPos*...
                                  aroundcenterTablePointsPos(:,j-1);
                aroundcenterTablePointsNeg(:,j) = RotNeg*...
                                  aroundcenterTablePointsNeg(:,j-1);
                              
                end
            aroundcenterTablePointsPos = aroundcenterTablePointsPos+centerTable';
            aroundcenterTablePointsNeg = aroundcenterTablePointsNeg+centerTable';

            distancesPos = zeros(N,1);
            distancesNeg = zeros(N,1);

            for j = 1:N
                distancesPos(j) = norm(aroundcenterTablePointsPos(:,j)-NewCenterAbsuluteAxis(1:2));
                distancesNeg(j) = norm(aroundcenterTablePointsNeg(:,j)-NewCenterAbsuluteAxis(1:2));

            end
            indexMinPos = find(distancesPos == min(distancesPos));
            indexMinNeg = find(distancesNeg == min(distancesNeg));
            
            if indexMinPos <= indexMinNeg
                aroundcenterTablePoints = aroundcenterTablePointsPos(:,1:indexMinPos);
            else
                aroundcenterTablePoints = aroundcenterTablePointsNeg(:,1:indexMinNeg);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%
            controller1 = robotics.PurePursuit('DesiredLinearVelocity',...
                                               0.05,... %0.02 %0.05
                                                'LookaheadDistance',...
                                               0.02,...
                                               'MaxAngularVelocity',...
                                               0.08,... %0.02 %0.02
                                               'Waypoints',...
                                               aroundcenterTablePoints');
                                           
% % %            path2 = [youbotPos(1:2);NewCenterRef(1:2)'];                             
% % %            controller2 = robotics.PurePursuit('DesiredLinearVelocity',...
% % %                                                0.02,... %0.02
% % %                                                 'LookaheadDistance',...
% % %                                                0.02,...
% % %                                                'MaxAngularVelocity',...
% % %                                                0.02,... %0.02
% % %                                                'Waypoints',...
% % %                                                path2);                                
           goalRadius1 = 0.2; % 0.3
           distanceToGoal1 = norm(youbotPos(1:2)'-aroundcenterTablePoints(:,end));
% % %            goalRadius2 = 0.3; % 0.3 ca fonctionne bien quand meme
% % %            distanceToGoal2 = norm(youbotPos(1:2)'-NewCenterRef(1:2));
           
           while( distanceToGoal1 > goalRadius1)
            start_loop1 = tic;
            % Compute the controller outputs, i.e., the inputs to the robot
            % Simulate the robot using the controller outputs.
            % exprimer robotvel(1) and robotvel(2) and orientation (par
            % rapport robot)
            % en fonction de v et omega (par rapprot axes absolus)
            
% 
            if indexMinPos > indexMinNeg
                [v, omega] = controller1([youbotPos(1:2), wrapTo2Pi(wrapTo2Pi(youbotEuler(3))-pi/2)]);
                v = -v;
            else
                
               [v, omega] = controller1([youbotPos(1:2),wrapTo2Pi(wrapTo2Pi(wrapTo2Pi(youbotEuler(3))+pi)-pi/2)]);

            end


            h = youbot_drive(vrep, h, v, 0, omega);
        
            % Extract current location information ([X,Y]) from the current pose of the
            [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res);
            [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res);              
            distanceToGoal1 = norm(youbotPos(1:2)' - aroundcenterTablePoints(:,end));
            ellapsed = toc(start_loop1);
            remaining = timestep - ellapsed;
            % time control
            if remaining > 0
                pause(min(remaining, .01));
            end
            
           end
           h = youbot_init(vrep, id);
           h = youbot_hokuyo_init(vrep, h);
           h = youbot_drive(vrep, h, 0, 0, 0);
            pause(0.01);

% % %            if (distanceToGoal1 <= goalRadius1)
% % %                while (distanceToGoal2 > goalRadius2)
% % %                    start_loop2 = tic;
% % %                    [v, ~] = controller2([youbotPos(1:2),0]);
% % %                    h = youbot_drive(vrep, h, 0, -v, 0);
% % %                    % Extract current location information ([X,Y]) from the current pose of the
% % %                    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
% % %                    vrchk(vrep, res);              
% % %                    % distanceToGoal2 = norm(youbotPos(1:2)' - NewCenterRef(1:2));
% % %                    distanceToGoal2 = norm(youbotPos(1:2)' - centerTable')-0.4;
% % %                    ellapsed = toc(start_loop2);
% % %                    remaining = timestep - ellapsed;
% % %                     % time control
% % %                    if remaining > 0
% % %                        pause(min(remaining, .01));
% % %                    end
% % %                end
% % %            end
% % %            pause(0.01);
% % %            h = youbot_init(vrep, id);
% % %            h = youbot_hokuyo_init(vrep, h);
% % %            h = youbot_drive(vrep, h, 0, 0, 0);
           i=1;
            
        else
            if i < length(AngleSensor)
                disp('Object Not Found');
                disp('Changing orientation of  XYZ sensor ...');
                i=i+1;
            
            else
                disp('Object Not Found after scanning all angles')
                stateObject = 'Unpicked';
                 break
            end
        end
   end
end