classdef drone < handle

    % drone class, contains an array of its coordinates
    
    properties (Access=private)
        id=1         % id of the drone
        helix=4      % number of helix
        speed=1.0    % max speed
        weight=500   % gramms
        comRange=100 % wireless communication range
        
        %Propriétés dynamiques
        coords = point(0.0,0.0,0.0) %Coordonnées du drone
        convexHull = convexHull2D()
        tmpConvexHull = convexHull2D()
        intersection = []
        goalPoint = point(0.0,0.0,0.0) % Point de destination commun aux drones
        poly = []
        polyInter = []
        
    end
    methods
        function obj=drone(id,h,s,w,p,gp,range)
            if nargin >= 1
                obj.id = id;
            end
            if nargin >= 2
                obj.helix = h;
            end
            if nargin >= 3
                obj.speed = s;
            end
            if nargin >= 4
                obj.weight = w;
            end
            if nargin >= 5
                obj.coords = p;
            end
            if nargin >= 6
                obj.goalPoint = gp;
            end
            if nargin >=7
                obj.comRange = range;
            end
        end
        function ret=getHelice(obj)
            ret = obj.helix;
        end
        function obj=setHelice(obj,value)
            obj.helice=value;
        end
        function ret=heliceCarry(obj)
            ret = obj.weight/obj.helix;
        end
        function ret=getPosition(obj)
            ret = obj.coords;
        end
        function obj=setPosition(obj, pos)
            obj.coords = pos; 
        end
        function obj=setConvexHull2D(obj,cv)
            obj.convexHull = cv;
        end
        function ret=getConvexHull2D(obj)
            ret = obj.convexHull;
        end
        function obj=setTmpConvexHull2D(obj,cv)
            obj.tmpConvexHull = cv; 
        end
        function ret=getTmpConvexHull2D(obj)
            ret = obj.tmpConvexHull;
        end
        function obj=setGoalPoint(obj,p)
            obj.goalPoint = p;
        end
        function ret=getGoalPoint(obj)
            ret = obj.goalPoint;
        end
        function ret=getComRange(obj)
            ret = obj.comRange;
        end
        function obj=setComRange(obj,r)
            obj.comRange = r;
        end
        function obj=setPolytope(obj,polytope)
            obj.poly = polytope;
        end
        function ret=getPolytope(obj)
            ret = obj.poly;
        end
        function ret=getPolyInter(obj)
            ret = obj.polyInter;
        end
        function obj=setPolyInter(obj,poly)
            obj.polyInter = poly;
        end
    end
end