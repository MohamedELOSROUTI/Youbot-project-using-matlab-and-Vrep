function pts = scanXYZ(id,vrep,h,angleSensor, scanAngle)
    if id < 0
            disp('Failed connecting to remote API server. Exiting.');
            vrep.delete();
            return;
    end
%% Read data from the depth camera (Hokuyo)
            % Reading a 3D image costs a lot to VREP (it has to simulate the image). It also requires a lot of 
            % bandwidth, and processing a 3D point cloud (for instance, to find one of the boxes or cylinders that 
            % the robot has to grasp) will take a long time in MATLAB. In general, you will only want to capture a 3D 
            % image at specific times, for instance when you believe you're facing one of the tables.

            % Reduce the view angle to pi/8 in order to better see the objects. Do it only once. 
            % ^^^^^^     ^^^^^^^^^^    ^^^^                                     ^^^^^^^^^^^^^^^ 
            % simxSetFloatSignal                                                simx_opmode_oneshot_wait
            %            |
            %            rgbd_sensor_scan_angle
            % The depth camera has a limited number of rays that gather information. If this number is concentrated 
            % on a smaller angle, the resolution is better. pi/8 has been determined by experimentation. 
            res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', scanAngle, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
                        vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, angleSensor], vrep.simx_opmode_oneshot);

            % Ask the sensor to turn itself on, take A SINGLE POINT CLOUD, and turn itself off again. 
            % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
            %         |
            %         handle_xyz_sensor
            res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);

            % Then retrieve the last point cloud the depth sensor took.
            % If you were to try to capture multiple images in a row, try other values than 
            % vrep.simx_opmode_oneshot_wait. 
            fprintf('Capturing a point cloud...\n');
            pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
            % Each column of pts has [x;y;z;distancetosensor]. However, plot3 does not have the same frame of reference as 
            % the output data. To get a correct plot, you should invert the y and z dimensions. 
    
            % Here, we only keep points within 1 meter, to focus on the table. 
            pts = pts(1:3, pts(4, :) < 1.5);
            
            
end