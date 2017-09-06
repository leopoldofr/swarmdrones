classdef obstacle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=private)
        polytope = [];
        type = "none";
    end
    
    methods
        function obj=obstacle(type,width)
           if nargin == 0 || type == ""
              type = "cube"; 
           end
           if nargin == 1
               width = 500;
           end
           switch type
               
               case "cube"
                  structure = createCube;
                  obj.type = type;
               case "octa"
                   structure = createOctahedron;
                   obj.type = type;
               case "soccer"
                   structure = createSoccerBall;
                   obj.type = type; 
               otherwise
                   structure = createCube;
                   obj.type = "cube";
           
           end
           
           %Prepare the center
           x = 1 + rand(1)* 4000;
           y = 1 + rand(1)* 2000;
           z = 1;
           
           %We apply the transformations
           vertices = structure.vertices;
           T = createTranslation3d(x,y,z);
           S = createScaling3d(width);
           tr = composeTransforms3d(S,T);
           vertices = transformPoint3d(vertices, tr);
           obj.polytope = vertices;
            
        end
    
        function ret =getPolytope(obj)
    
            ret = obj.polytope;
    
        end
    end
    
end

