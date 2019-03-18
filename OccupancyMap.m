classdef OccupancyMap < handle
    %OCCUPANCYMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Map
        MapRes
        X_axis
        Y_axis
    end
    
    properties(Constant)
        Unknown = 0;
        Free = 1;
        Wall = 2;
    end
    
    methods
        function obj = OccupancyMap(size, res)
            obj.Map = zeros(size);
            obj.MapRes = res;
            
            obj.X_axis = [0 (size(1)-1)*res];
            obj.Y_axis = [0 (size(2)-1)*res];
        end
        
        function addPoints(obj, x, y, value)
            indexes = sub2ind(size(obj.Map), x, y);
            obj.Map( indexes(obj.Map(sub2ind(size(obj.Map), x, y)) < value) ) = value;
        end
        
        function show(obj)
            imagesc(obj.X_axis, obj.Y_axis, obj.Map);
            set(gca,'xaxisLocation','top')
        end
    end
    
end

