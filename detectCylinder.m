function [center, Found] = detectCylinder(pts)
    ptCloud = pointCloud(pts'); % require format Mx3 INSTEAD of 3xM
%     pcshow(ptCloud);
    Found = 1;
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    title('Original Point Cloud');


    referenceVector = [0,1,0];
    maxDistance = 0.005;
    tic;
    [model, inlierIndices] = pcfitcylinder(ptCloud,...
                maxDistance,referenceVector,...
                'Confidence', 80);
    i=1;
    if size(inlierIndices,1) < 40
        Found = 0;
    end
    while model.Radius > 0.025 && Found == 1

        try
        if toc > 20
            break;
        end
        [model, inlierIndices] = pcfitcylinder(ptCloud,...
                maxDistance,referenceVector,...
                'Confidence', 80,...
                'MaxNumTrials',10000);
            i=i+1;
            
            if i == 15 || size(inlierIndices,1) < 50
                Found = 0;
                break;
            end
            
        catch
            
            Found = 0;
            break;
        end
    end
    
    % disp(i)
%     hold on
%     plot(model)
    center = model.Center;
  
end
    