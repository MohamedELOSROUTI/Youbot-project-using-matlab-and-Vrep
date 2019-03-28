classdef OccupancyMap < handle
    %OCCUPANCYMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(SetAccess=private)
        Map          % Map matrix
        MapRes       % Occupancy map resolution
        X_axis       % Plot X axis
        Y_axis       % Plot Y axis
        
        changed      % Indicates if the occupancy map has changed
    end
    
    properties(Constant)
        Unknown = 0;
        Free = 1;
        Wall = 2;
    end
    
    methods
        function newTarget = findNewTarget(obj,currentPosIndex)
            % return the closest index to currentPosIndex with 0 values
            [index0(:,1),index0(:,2)]=find(obj.Map==obj.Unknown);
            [~,I]=min(vecnorm(currentPosIndex-index0,2,2));
            newTarget = index0(I,:);
        end
        
        function obj = OccupancyMap(size, res)
            obj.Map = zeros(size);
            obj.MapRes = res;
            
            obj.X_axis = [0 (size(1)-1)*res];
            obj.Y_axis = [0 (size(2)-1)*res];
            obj.changed = false;
        end
        
        % Add multiple points with a certain value to the occupancy map
        function addPoints(obj, x, y, value)
            indexes = sub2ind(size(obj.Map), x, y);
            
            [r, c] = size(indexes(obj.Map(sub2ind(size(obj.Map), x, y)) < value));
            if r ~= 0 && ~obj.changed
                obj.changed = true;
            end
            
            obj.Map( indexes(obj.Map(sub2ind(size(obj.Map), x, y)) < value) ) = value;
        end
        
        % Plot the occupancy map according the X and Y axis defined in the
        % constructor
        function plot(obj)
            imagesc(obj.X_axis, obj.Y_axis, obj.Map);
            set(gca,'xaxisLocation','top')
        end
        
        
        % Return map with only known obstacles represented by 1
        % according to RTB Dstar map definition
        function m = getCostmap(obj)
            obj.changed = false;
            
            m = ones(size(obj.Map));
            m(obj.Map == 2) = Inf;
            
        
        end
        
    end
    
end

