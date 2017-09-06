% ### Program to simulate drone swarm moving together according to Javier
% Alonso-Mora et al's algorithm
% (https://web.stanford.edu/~schwager/MyPapers/Alonso-MoraEtAlICRA16FormationNavigation.pdf)

% ### Warning:
% - On an unix computer (my distrib is Linux Mint) you must launch Matlab with the -softwareopengl option or
% you will have stange warnings while rendering the drone with plot()

% The plugins installed are geom3d and intersectionHull. I modified the
% convexHull one

tic
startSimulation();
toc

function []= startSimulation()

    %Main function
    nbDrones = 8;                      % The number of drones for the simulation
    simulationFrames = 20;             % Time of simulation, in frames
    rangeDrones = 2000;                % Viewport size rangeDrones x rangeDrones
    goalPoint = point( 3500, 1000, 0); % The objective point to reach
    comRange = 1000;                   % Wireless communication range for a drone
    
    clc();
    delete(findall(0,'Type','figure'))
    allDrones=init(nbDrones, goalPoint,rangeDrones, comRange);
    
    drawWholeSimulation(simulationFrames, allDrones, rangeDrones, comRange, goalPoint)
    
end

function allDrones=init(nbDrones, goalPoint, rangeDrones, comRange)
    %Initialize the drone swarm with nbDrones drones and random coordinates
    
    allDrones=cell(1,nbDrones);
    offset = 0.3;
    for i=1:nbDrones
        p = point(rand()*rangeDrones * offset + (offset*rangeDrones), rand()*rangeDrones * offset + (offset * rangeDrones), 1);%rand()*rangeDrones * offset + (offset*rangeDrones));
        allDrones{i}=drone(i,4,1.0,1000, p, goalPoint,comRange);
    end
    
end

function [] = drawWholeSimulation(simulationFrames, allDrones, rangeDrones, comRange, goal)
    %Run and draw the whole simulation
    
    %Options parameters for the simulation
    renderComRange = true;                          %Render the circle for the range communication of all drones
    randomizeComDrones = true;                      %Add Randomization to the communication range of each drone
    randomizeComDronesPurcent = [comRange, 0.1];    %[avg, purcent] plus purcent est élevé, plus il y a de perturbations
    
    %Delete the previous figures
    delete(findall(0,'Type','figure'));
    fig1 = figure;
    set(fig1, 'Position', get(0,'Screensize'))
    
    %Initialisation
    obstacles = generateObstacles(5);
   
    for i=1:simulationFrames

        %Randomize the communication range at each frame
        if randomizeComDrones == true
            randomComDrones(allDrones, randomizeComDronesPurcent);
        end
        
        %Algorithm 1 => Convex Hull
        calculateConvexHull(allDrones);     
        
        %Algorithm 2 => Intersection of the vision of each drone
        calculateIntersection(allDrones);
        
        %Last step, calculate the optimal formation
        % TODO
        
        drawSingleFrame(allDrones, rangeDrones,renderComRange, goal, obstacles);
        pause(0.15); % On my current computer a short pause is necessary to render the convexHull result. It doesn't render if there is no pause  
        
        clf()
        moveDrones(allDrones);
        
        reinitPolytopesDrones(allDrones);
        
    end
    
end

function h = drawSingleFrame(allDrones, rangeDrones, renderComRange, goal, obstacles)
    %Draw a single frame to display drones
    %It displays the convex hull of the drones, the intersection hull
    %It can display the communication range
    
    nbdrones = size(allDrones,2);
    h = [];
    colors = ['m' 'c' 'b' 'r' 'k' 'g' 'r'];
    for i=1:nbdrones
        
        pos = allDrones{i}.getPosition();
        p = pos.getCoords();
        b = mod(i,size(colors));
        b = b(2);
        if b == 0
            b = 1;
        end
        color = strcat(colors(b),'o');
        h = plot(p(1), p(2), color);
        
        hold on;
        
    end
    
    %Prepare the plot (title, labels, xlim,...)
    xlabel("coord X");
    ylabel("coord Y");
    zlabel("coord Z");
    xlim([0 rangeDrones*2]);
    ylim([0 rangeDrones]);
    zlim([0 rangeDrones/2]);
    pbaspect([2 1 0.5]); % Ratio of the axis for a better display
    title("Swarm drone moving (in progress)");
    grid on;
    zoom on;
    
    %Render each drone as a point
    for i =1:size(allDrones,2)
        cv = allDrones{i}.getConvexHull2D();
        X = cv.getX();
        Y = cv.getY();
        K = cv.getK();

        plot(X(K),Y(K),'r-',X,Y,'b*')
        
        %Render (if option renderComRange is set to true) the communication
        %circle around each drone
        if renderComRange == true
            x = allDrones{i}.getPosition().getX();
            y = allDrones{i}.getPosition().getY();
            r = allDrones{i}.getComRange();
            ang=0:0.01:2*pi; 
            xp=r*cos(ang);
            yp=r*sin(ang);
            plot(x+xp,y+yp, 'k--');
        end
        
        %Render the current polytope of each drone
        V = allDrones{i}.getPolytope();
        scatter3(V(:,1),V(:,2),V(:,3),'b');
        alpha(.2);
        
    end
       
    %Rendering the goal point
    plot(goal.getX(),goal.getY(), '*r');
    text(goal.getX() + 20,goal.getY(), 'Objectif');
    
    %Uncomment the following line to view the viewport in 3D
    %view(3)
    
     displayObstacles(obstacles)
end

function [] = moveDrones(allDrones)
    %Move randomly the drones for now
    %It chooses a random X and Y and move the drones
    
    nbDrones = size(allDrones,2);
    for i=1:nbDrones
       drone = allDrones{i};
       coords = drone.getPosition().getCoords();
       coords2 = coords +[(rand()-0.5)*50 (rand()-0.5)*50 1]; 
       p2 = point(coords2(1), coords2(2), coords2(3));
       drone.setPosition(p2);
       
    end
end

function calculateConvexHull(allDrones)
    %Calculate the convex hull for all drones
    %allDrones is a drone array
    nbDrones = size(allDrones,2);
    for i = 1:nbDrones
        cd = allDrones{i}.getPosition().getCoords();
        init_cv_hull = convexHull2D(1,cd(1),cd(2));
        init_tmp_cv_hull = convexHull2D();
        allDrones{i}.setConvexHull2D(init_cv_hull);
        allDrones{i}.setTmpConvexHull2D(init_tmp_cv_hull);
    end
    
    for i = 1:nbDrones
        sendConvexHull(allDrones, i);
        receiveConvexHull(allDrones, i);
    end
    
end

function sendConvexHull(allDrones,i)
    %A drone send it's difference between the k-1 an k nth convexhulls
    %differences
    tmpConvexHull = allDrones{i}.getTmpConvexHull2D();
    convexHull = allDrones{i}.getConvexHull2D();
    
    %Calculate the difference    
    tmp = diff(convexHull,tmpConvexHull);
    tmp = convexHull2D(1,tmp(:,1),tmp(:,2));
    K = getDronesInRange(allDrones,i);
    for j=K
       allDrones{j}.setTmpConvexHull2D(tmp);
    end

end

function receiveConvexHull(allDrones, i)
    
    %Each drone is going to calculate its update
    K = getDronesInRange(allDrones,i);
    for j = K
           
        tcv = allDrones{j}.getTmpConvexHull2D();
        cv = allDrones{j}.getConvexHull2D();
        diffcv = diff(cv, tcv);
            
        %le drone i calcule sa mise à jour et update sa convex hull
        tmp = mergeHull(allDrones{i}.getConvexHull2D(), diffcv);
        allDrones{i}.setConvexHull2D(tmp);
    end
end

 function ret = diff(cv,tcv)
    % Function that calculates the difference between two convexHull2D
    % object an returns an array of its X Y, both columns vectors
    % It returns the new values
    XY1 = cat(2,cv.getX(),cv.getY());
    XY2 = cat(2,tcv.getX(),tcv.getY());
    if(size(XY2,1) == 0)
        ret = cat(2,cv.getX(), cv.getY());
    else
        ret = setdiff(XY1,XY2,'rows');
    end
         
 end
   
 function ret = mergeHull(current,old)
    %Takes an update and merge it
    %An update is just a two entry array of columns X,Y (parameter old)
    X = cat(1,current.getX(), old(:,1));
    Y = cat(1,current.getY(), old(:,2));
    
    %Here we remove all dupplicates
    XY = unique(cat(2,X,Y), 'rows');
    X = XY(:,1);
    Y = XY(:,2);
    
    if size(X,1) <= 2
    
        ret = convexHull2D(size(X,1), X, Y);
    
    elseif size(X,1) > 2
    
        K = convhull(X,Y);
        ret = convexHull2D(K,X,Y);
    
    end
 end
 
 function K = getDronesInRange(allDrones, d)
    %get indices of drones in the range of the drone number d in 3D
    %return an array of indices
    %The function does not include the drone d in the array
    
    K = zeros(1,8);
    c1 = allDrones{d}.getPosition();

    for i=1:size(allDrones,2)
        
        if not(i == d)
            c2 = allDrones{i}.getPosition();
            x = power(c2.getX() - c1.getX(),2);
            y = power(c2.getY() - c1.getY(),2);
            z = power(c2.getZ() - c1.getZ(),2);
            dist = sqrt(x+y+z);
            if dist < (allDrones{d}.getComRange() + allDrones{i}.getComRange())
                K(i) = i;
            end
        end
    end
    K = K(K~=0);
 end
 
 function randomComDrones(allDrones, rcdp)
    %Randomize the drones communications according to the parameter rcdp
    for i=1:size(allDrones,2)
        r = rcdp(1) + (rcdp(1) * rcdp(2)) - rand()*(rcdp(1) * rcdp(2) * 2);
        allDrones{i}.setComRange(r);
    end
 end
 
 function []= calculateIntersection(allDrones)
 
     %Calculate the intersection of the polytope for each drone
     for j=1:8

         for i=1:size(allDrones,2)             
             sendPolytopeDrones(allDrones, i);             
             receivePolytopeDrones(allDrones, i);         
         end
     end
    
 end

 function []=sendPolytopeDrones(allDrones, i)
    %Function that will calculate the 
    
    if size(allDrones{i}.getPolytope(),1) == 0
        
        [vertices, ~, ~] = createSoccerBall; % '~' means that this return value won't be useful
        T = createTranslation3d(allDrones{i}.getPosition.getX(),allDrones{i}.getPosition().getY(),1);
        S = createScaling3d(allDrones{i}.getComRange());
        tr = composeTransforms3d(S,T);
        vertices = transformPoint3d(vertices,tr);
        allDrones{i}.setPolytope(vertices);
        
    end
        
    vertices = allDrones{i}.getPolytope();
    
    K = getDronesInRange(allDrones, i);
    for k=K
        allDrones{k}.setPolyInter(vertices);
    end
 
 end
 
 function []=receivePolytopeDrones(allDrones, i)
    
    %A drone will compute it's own polytope with the temporary one and then
    %it will store the new one as it's new polytope
    for k=getDronesInRange(allDrones,i)
        if(size(allDrones{k}.getPolytope(),1) == 0)
            [V, ~, ~] = createSoccerBall;
            T = createTranslation3d(allDrones{k}.getPosition.getX(),allDrones{k}.getPosition().getY(),1);
            S = createScaling3d(allDrones{k}.getComRange());
            tr = composeTransforms3d(S,T);
            V = transformPoint3d(V,tr);
            allDrones{k}.setPolytope(V);
        end
        
        V = intersectionHull('vert', allDrones{k}.getPolytope(), 'vert', allDrones{k}.getPolyInter());
        V = V.vert;
        allDrones{k}.setPolytope(V);
        V = intersectionHull('vert', allDrones{i}.getPolytope(), 'vert', V);
        V = unique(V.vert,'rows');

        allDrones{i}.setPolytope(V);
        
    end
 end
 
 function reinitPolytopesDrones(allDrones)
    %Function that reinitialize the drones polytopes for the next frame
 
    for i=1:size(allDrones,2)
         allDrones{i}.setPolytope([]);
         allDrones{i}.setPolyInter([]);
     end
 
 end
 
 function ret =generateObstacles(nb)
    %In construction...
 
    %Configuration
    range = 300;
 
    %We randomly choose one of these for a single obstacle
    tab = ["cube","octa","soccer"];
    
    if nb <= 0
        ret = [];
    else
    
        ret = cell(nb);

        for i=1:nb
            x = round(1+rand(1)*(size(tab,2)-1));
            type = tab{x};
            ret{i} = obstacle(type, range);

        end
     end
 end
 
 function displayObstacles(obstacles)
    %Display all obstacles on the plot

    for i=1:size(obstacles,2)
        
        plotpoly(obstacles{i}.getPolytope());
        
    end
 end
 
 
 
 
 