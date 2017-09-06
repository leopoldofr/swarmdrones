classdef point < handle
    %point base unit for the convex hull generation
    
    properties (Access = private)
        cX = 0.0
        cY = 0.0
        cZ = 0.0
    end
    
    methods
        function obj=point(x,y,z)
            obj.cX = x;
            obj.cY = y;
            obj.cZ = z;
        end
        function ret = getCoords(obj)
            ret = [obj.cX, obj.cY, obj.cZ];
        end
        function ret = getX(obj)
            ret = obj.cX;
        end
        function ret = getY(obj)
            ret = obj.cY;
        end
        function ret = getZ(obj)
            ret = obj.cZ;
        end
    end
end

