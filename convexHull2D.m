classdef convexHull2D < handle
    %convexHull2D object is used to store the K (index indices), X, Y of a
    %convhull previously calculated
    
    properties (Access=private)
        K = [];
        X = [];
        Y = [];
    end
    
    methods
        function obj=convexHull2D(K,X,Y)
            
            if nargin == 0
                obj.K = [];
                obj.X = [];
                obj.Y = [];
            elseif nargin == 3
                obj.K = K;
                obj.X = X;
                obj.Y = Y;
            else
                error("[!] Creating a convex Hull requires at least K,X,Y");
            end
        end
        function ret=getK(obj)
            ret = obj.K;
        end
        function ret=getX(obj)
            ret = obj.X;
        end
        function ret=getY(obj)
            ret = obj.Y;
        end
        function disp(obj)
            disp("K = " + obj.getK())
            disp("X = " + obj.getX())
            disp("Y = " + obj.getY())
            disp(" ")
        end
    end
    
end

