function takePicture(id,vrep,h,angleCamera)

%% Read data from the RGB camera
            % This starts the robot's camera to take a 2D picture of what the robot can see. 
            % Reading an image costs a lot to VREP (it has to simulate the image). It also requires a lot of bandwidth, 
            % and processing an image will take a long time in MATLAB. In general, you will only want to capture 
            % an image at specific times, for instance when you believe you're facing one of the tables or a basket.

            % Ask the sensor to turn itself on, take A SINGLE IMAGE, and turn itself off again. 
            % ^^^     ^^^^^^                ^^       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            % simxSetIntegerSignal          1        simx_opmode_oneshot_wait
            %         |
            %         handle_rgb_sensor
            
            vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, angleCamera], vrep.simx_opmode_oneshot);
            res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            
            % Then retrieve the last picture the camera took. The image must be in RGB (not gray scale). 
            %      ^^^^^^^^^^^^^^^^^^^^^^^^^     ^^^^^^                            ^^^
            %      simxGetVisionSensorImage2     h.rgbSensor                       0
            % If you were to try to capture multiple images in a row, try other values than 
            % vrep.simx_opmode_oneshot_wait. 
            fprintf('Capturing image...\n');
            [res, resolution, image] = vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            fprintf('Captured %i pixels (%i x %i).\n', resolution(1) * resolution(2), resolution(1), resolution(2));
            imshow(image);

end