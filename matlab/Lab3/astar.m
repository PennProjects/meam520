function [path, num_expanded] = astar(map, start, goal)
% ASTAR Find the shortest path from start to goal.
%   PATH = ASTAR(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix. Consecutive points in PATH should not be farther apart
%   than neighboring voxels in the map (e.g. if 5 consecutive points in
%   PATH are co-linear, don't simplify PATH by removing the 3 intermediate points).
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration
%
% AUTHOR
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu)

%% Prep Code

astar = true;
path = [];
num_expanded = 0;


robot.d1 = 76.2;
robot.a2 = 146.05;
robot.a3 = 187.325;
robot.d5 = 68;
robot.lg = 35;
robot.lowerLim = [-1.4000 -1.2000 -1.8000 -1.9000 -2 -15];
robot.upperLim = [1.4000 1.4000 1.7000 1.7000 1.5000 30];
robot.d4 = 34;

map = getCMap(map,robot,[0.1,0.1,0.1],25);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Settings

% Planning is in 3D C-space (we choose to ignore theta4 and theta5):
fullStart = start; fullGoal = goal;
start = start(1:3); goal = goal(1:3);

% Save info about the world to the map structure.
map.maxy = size(map.occgrid,1);
map.maxx = size(map.occgrid,2);
map.maxz = size(map.occgrid,3);
map.maxima = [map.maxy;map.maxx;map.maxz];
map.num_nodes = map.maxy*map.maxx*map.maxz;
map.num_free = nnz(~map.occgrid);

% Variables for calculating the cost of travel to neighbors.
map.info.xy = map.res_th123(1); map.info.z = map.res_th123(3);
map.info.ratio = map.info.z/map.info.xy;

map.info.neighborlists = containers.Map('KeyType','int32','ValueType','any');
map.info.neighborcostlists = containers.Map('KeyType','int32','ValueType','any');

%% Check for valid start and goal

valid = validStartGoal(map,start,goal);

if ~valid
    return
end

%% Data Structures

% Initialize obstacles matrix - infinity on obstacles, zero elsewhere.
%   Arranged by index.
world.obstacles = reshape(Inf*(map.occgrid==1),[],1);
world.obstacles(isnan(world.obstacles)) = 0;

% Initial 1D cost matrix - all points initially have infinite cost -
%   and matrix to store previous cost. Arranged by index.
world.cost = Inf*ones(size(world.obstacles));
world.cost_prev = Inf*ones(size(world.cost));

% Store parent information in a nx1 vector where the value corresponds to
%   the index of the parent node (initially all zero since no zero index).
world.parent = zeros(size(world.cost));

% Store status of explored nodes. 1 for not explored and 0 for explored.
world.explored = ones(size(world.cost));

% Calculate heuristics.
world.heuristics = heuristic(map, goal, astar);

%CHANGE CHANGE CHANGE
world.neighbors2search = world.cost;

%% Implement Dijkstra's Algorithm or A*

% Find the index corresponding to the start position.
startind = pos2ind(map, start, 1);

% Find the index of the goal node.
goalind = pos2ind(map, goal, 1);

% Set the cost of the start position to 0 and the parent index of the start
%   node to itself. Add the start position to the path.
world.cost(startind) = 0;
world.cost_prev(startind) = 0;
world.parent(startind) = startind;

% Set the current node as the start node and initialize minimum cost as
%   zero. Initialize array of current indices to look at.
currind = startind;

% Continue expanding nodes as long as we haven't found the goal node.
while (world.parent(goalind) == 0)

    % Set current node as explored in both explored and neighbors list.
    world.explored(currind) = 0;
    world.neighbors2search(currind) = Inf;

    % Get list of neighbor indices and their associated costs.
    [neighbors, neighborcosts] = getNeighbors(currind, map.maxima, map.info);


    % Add distance cost if this will result in a cheaper result but leave
    %   infinity cost if it's an obstacle.
    world.cost(neighbors) = max(min(world.cost(currind) + neighborcosts, ...
                                    world.cost_prev(neighbors)), ...
                                world.obstacles(neighbors));

    % Add cost of only those nodes that have changed to the list of nodes
    %   to be searched.
    world.neighbors2search(neighbors(world.explored(neighbors) ~= 0)) = ...
        world.cost(neighbors(world.explored(neighbors) ~= 0)) ...
        + world.heuristics(neighbors(world.explored(neighbors) ~= 0));

    % Change parent of nodes whose cost changed. We retain the original
    %   parents by elementwise multiplying with the matrix of costs that
    %   stayed the same and then sum with the parent for the costs that
    %   changed, which will have zero for all those that remained. The
    %   first multiplicaton accounts for costs that decreased from already
    %   having a parent.
    world.parent(neighbors) = ...
        world.parent(neighbors).*(world.cost(neighbors) == world.cost_prev(neighbors)) + ...
        currind*(world.cost(neighbors) ~= world.cost_prev(neighbors));

    % Adjust the previous costs vector to maintain continuity.
    world.cost_prev(neighbors) = world.cost(neighbors);

    % Get a vector of all nodes with the next minimum cost
    currind = getNextNode(world.neighbors2search);

    % Update number of expanded nodes
    num_expanded = num_expanded + 1;

    if (num_expanded > map.num_nodes)
        fprintf('\n\n Explored all nodes. No path found. \n\n');
        return
    end

end


%% Generate and plot the path

% If we got this far without returning, a path exists from the start to the
%   goal. We now extract it from the parent vector.

path = getPath(map, start, goal, world.parent);
path = [path,zeros(size(path,1),3)];
path = [fullStart;path;fullGoal]

% plot_path(map, path);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HELPER FUNCTIONS


function [neighbors, neighborcosts] = getNeighbors(currind, maxima, info)
% GETNEIGHBORS - find the indices and costs of nodes that are neighbors to
%    the current node being analyzed. Accounts for faces, corners, etc.
%
% INPUTS:
%   currind: index of current node under observation.
%   maxima:  3x1 vector of max values, defining the dimension of the array.
%       Ordered as (maxy;maxx;maxz)
%   info:    a structure containing distance cost info.
%
% OUTPUTS:
%   neighbors:     a Nx1 matrix of neighbors. At minimum it will be 8 items
%       (corners), at max it will be 27. The node itself is included.
%   neighborcosts: a Nx1 matrix of costs corresponding to the neighbor
%       nodes. Indices match, and cost for the node itself will be zero.

    % Get location of current node
    [J,I,K] = ind2sub(maxima, currind);
    currjik = [J;I;K];

    % Determine whether we're at a corner, an edge, a face, or in the
    % space. The edge case where a dimension is reduced is accounted for by
    % the last statement.
    idval = [1,10,100]*(currjik <= [1;1;1]) ...
                + [2,20,200]*(currjik >= maxima) ...
                + [4,40,400]*(currjik > [1;1;1] & currjik < maxima);

    if ~isKey(info.neighborlists, idval)
        vals = [0,1].*(currjik <= [1;1;1]) + [-1,0].*(currjik >= maxima) ...
            + [-1,1].*(currjik > [1;1;1] & currjik < maxima) ...
            - [-1,1].*(maxima == [1;1;1;]);

        % Transform from a 1x2 vector in vals to a 1x2 or 1x3 vector, adding in
        % the cost to move. The cost to move in y is 1 since index numbering
        % goes down the columns. The cost to move in x is maxy, since we have
        % to move a full column over. The cost to move in z is maxx*maxy.
        jvals = vals(1,1):vals(1,2);
        ivals = maxima(1)*(vals(2,1):vals(2,2));
        kvals = maxima(1)*maxima(2)*(vals(3,1):vals(3,2));

        % Use the cost matrices to compute the cost of motion along an axis.
        costjvals = info.xy*(vals(1,1):vals(1,2));
        costivals = info.xy*(vals(2,1):vals(2,2));
        costkvals =  info.z*(vals(3,1):vals(3,2));

        % Compute the neighbors by adding column matrices to row matrices.
        % Credit to Luca Scheuer for showing me this approach.
        flatvals = jvals' + ivals;
        cubevals = flatvals(:) + kvals;
        info.neighborlists(idval) = cubevals(:);

        % Apply the same approach to the cost, but square and square-root to
        % calculate distance.
        costflatvals = ((costjvals').^2 + costivals.^2).^0.5;
        costcubevals = ((costflatvals(:)).^2 + costkvals.^2).^0.5;
        info.neighborcostlists(idval) = costcubevals(:);
    end

    neighbors = info.neighborlists(idval) + currind;
    neighborcosts = info.neighborcostlists(idval);

end

function [distances] = heuristic(map, goal, astar)
% HEURISTIC - calculates the heuristic to apply to all the nodes to choose
%   the closes one. For Dijkstra this is zero, for A* its the Manhattan
%   distance. Although the Manhattan distance is not technically admissible
%   as a heuristic for a 26-connected grid, it produces paths with fewer
%   turns and significantly reduces run-time. Given the post-processing
%   that happens during the trajectory generation, this is considered ok.
%
% INPUTS:
%   map:   structure containing map info
%   goal:  (x,y,z) of the goal node.
%   astar: flag for whether A* algorithm is used (yes if astar=true).
% OUTPUTS:
%   distances: a Nx1 vector of distances from the indexed node to the goal

    % For all the logical indices.
    indices = [1:map.num_nodes]';

    if (astar)
        % Compute the euclidian distance heuristic.
        %distances = vecnorm(ind2pos(map,indices) - goal,2,2);
        % Calculate the Manhattan distance heuristic.
        distances = vecnorm(ind2pos(map,indices,1) - goal,1,2);
    else
        distances = 0*indices;
    end

end

function [nextinds] = getNextNode(neighbors2search)
% GETNEXTNODE - find the set of nodes that are next in line to be checked
%   (i.e. the set of unexplored nodes with the lowest cost).
%
% INPUTS:
%   cost:       the Nx1 vector of costs by index.
%   heuristics: the Nx1 vector of heuristics by index.
%   explored:   the Nx1 vector of explored state by index (0 explored, 1 not)
%   currinds:   the previous set of nodes we were checking.
%
% OUTPUTS:
%   nextinds: a vector of indices of the nodes that should be considered
%       next. Only empty if map has been searched completely w/out success.
%   err:      a flag for nextinds being an empty set.

        err = 0;

        % Find the minimum cost of an unexplored node.
        [~,nextinds] = min(neighbors2search);

end

function [path] = getPath(map, start, goal, parent)
% GETPATH - trace the parent of each node back from the goal to the start
%   to find the optimal path located by Dijkstra or A*.
%
% INPUTS:
%   map:    a struct with map info
%   start:  (x,y,z) of the start node
%   goal:   (x,y,z) of the goal node
%   parent: an Nx1 vector of parents for each node.
%
% OUTPUTS:
%   path: an Nx3 array of nodes corresponding to the path from start to
%       goal

    % Add the goal node to the path
    path = goal;
    pathind = pos2ind(map, goal, 1);

    % Find the snapped node the goal corresponds to
    pathnode = ind2pos(map, pathind, 1);

    % Until we reach the node that is its own parent (i.e. the snapped
    %   start node)
    while (parent(pathind) ~= pathind)
        % Add this node to the path.
        path = [pathnode;path];

        % Switch to the parent node
        pathind = parent(pathind);
        pathnode = ind2pos(map, pathind, 1);
    end

    % Add the actual start node to the path
    path = [start; path];
    % Move path nodes from their snapped corners to the middle of the
    % voxels they were in.
    % path(2:(end-1),:) = path(2:(end-1),:)+ 1/2*map.res_th123;
    % THIS WAS DONE IN CARTESIAN BUT MAKES NO SENSE IN CSPACE
end

function [valid] = validStartGoal(map, start, goal)
% VALIDSTARTGOAL checks for valid start and goal positions, both for
%   positions outside the map and ones in obstacles. Invalid message will
%   be printed and the main loop will exit when the error flag is passed.
%
% INPUTS:
%   map: a structure containing map details
%   start: (x,y,z) of the start position
%   goal: (x,y,z) of the goal position
%
% OUTPUTS
%   valid: a flag for validity - 1 for valid or 0 for not.

    % Assume valid
    valid = 1;

    % Find the voxel corresponding to the start position.
    startjik = pos2sub(map, start, 1);

    % If the voxel is outside the map, return error
    if nnz((startjik < 1)|(startjik > map.maxima')) > 0
        fprintf("\nThe start position you requested is outside the map.\n");
        fprintf("\nPlease try again.\n");

        valid = 0;

        return
    end

    % Find the voxel of the goal node.
    goaljik = pos2sub(map, goal, 1);

    % If the voxel is outside the map, return error
    if nnz((goaljik < 1)|(goaljik > map.maxima')) > 0

        fprintf("\nThe goal position you requested is outside the map.\n");
        fprintf("\nPlease try again.\n");

        valid = 0;

        return
    end

    % Check if start or end are in an obstacle
    startobstacle = map.occgrid(startjik(1), startjik(2),startjik(3));
    goalobstacle  = map.occgrid(goaljik(1),  goaljik(2), goaljik(3));

    % Print error message
    if (startobstacle || goalobstacle)
        if (startobstacle && ~goalobstacle)
            errorstring = "start position";
            isare = "is";
        elseif (~startobstacle && goalobstacle)
            errorstring = "goal position";
            isare = "is";
        else
            errorstring = "start and goal position";
            isare = "are";
        end

        fprintf("\nThe %s you requested %s located in an obstacle.\n\nPlease try again.\n", errorstring, isare);

        valid = 0;

        return
    end
end

function y = combvec(varargin)
%COMBVEC Create all combinations of vectors.
%
%  <a href="matlab:doc combvec">combvec</a>(A1,A2,...) takes any number of inputs A, where each Ai has
%  Ni columns, and return a matrix of (N1*N2*...) column vectors, where
%  the columns consist of all combinations found by combining one column
%  vector from each Ai.
%
%  For instance, here the four combinations of two 2-column matrices are
%  found.
%  
%    a1 = [1 2 3; 4 5 6];
%    a2 = [7 8; 9 10];
%    a3 = <a href="matlab:doc combvec">combvec</a>(a1,a2)

% Mark Beale, 12-15-93
% Copyright 1992-2010 The MathWorks, Inc.

if length(varargin) == 0
  y = [];
else
  y = varargin{1};
  for i=2:length(varargin)
    z = varargin{i};
    y = [copy_blocked(y,size(z,2)); copy_interleaved(z,size(y,2))];
  end
end
end

%=========================================================
function b = copy_blocked(m,n)

[mr,mc] = size(m);
b = zeros(mr,mc*n);
ind = 1:mc;
for i=[0:(n-1)]*mc
  b(:,ind+i) = m;
end
end
%=========================================================

function b = copy_interleaved(m,n)

[mr,mc] = size(m);
b = zeros(mr*n,mc);
ind = 1:mr;
for i=[0:(n-1)]*mr
  b(ind+i,:) = m;
end
b = reshape(b,mr,n*mc);
end

function [cmap] = getCMap(map, robot, res, mrgn)
% GETCMAP calculates an occupancy grid in 3D C space based on a map object
%   defining obstacles in cartesian space. Effectively, this function turns
%   a cartesian space map into a configuration space map.
%
% INPUTS
%   map  - a map struct containing a 1x6 boundary field defining the
%           boundaries of the space and an Nx6 obstacles array where each 
%           row defines an axis aligned bounding box (x1,y1,z1,x2,y2,z2). 
%           This struct is produced by the loadMap function.
%   robot - a struct containing the dimensions and joint limits of the
%           Lynx. This is stored in robot.mat.
%   res   - a 3x1 vector defining the resolution for theta1, theta2, and
%           theta3.
%   mrgn  - the safety margin added to the size of the box to account for
%           the zero thickness of the lines of the lynx.
%
% OUTPUTS
%   cmap - an equivalent map structure for c space, containing an occupancy
%           grid that can be used for graph searches.
%           cmap should have the following fields:
%
%       cmap.occgrid     - a 3D matrix containing 1 for occupied voxels and
%                           0 for free voxels. Note that the indexing is
%                           th2,th1,th3.
%       cmap.bound_th123 - 1x6 containing lower bounds for th1, th2, th3
%                           and then upper bounds for th1, th2, th3.
%       cmap.res_th123   - 1x3 containing the resolution for th1, th2, th3
%       cmap.res_th12    - 1x2 containing the resolution for th1, th2
%       cmap.res_th3     - 1x1 containing the resolution for th3
%       cmap.mrgn        - 1x1 containing the margin of safety for the Lynx
%                           (in mm).
%
% AUTHOR
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu)
%   NOTE ***** The ordering in this file is th2,th1,th3 as a leftover
%   from my MEAM620 implementation. But the student's code will be ordered
%   th1,th2,th3. 

%% Initialize C-space map
% cmap.boundary = [robot.lowerLim(1),robot.lowerLim(2),robot.lowerLim(3),...
%                  robot.upperLim(1),robot.upperLim(2),robot.upperLim(3)];
% This is the same field but needed in A*
cmap.bound_th123 = [robot.lowerLim(1),robot.lowerLim(2),robot.lowerLim(3),...
                    robot.upperLim(1),robot.upperLim(2),robot.upperLim(3)];
                
cmap.res_th123   = res;

% This structure follows the A* setup
cmap.res_th12 = cmap.res_th123(1:2);
cmap.res_th3 = cmap.res_th123(3);



cmap.mrgn = mrgn;

%% Asses the free C space

% Get vectors of options for each angle
th1s = [robot.lowerLim(1):res(1):robot.upperLim(1)]';
th2s = [robot.lowerLim(2):res(2):robot.upperLim(2)]';
th3s = [robot.lowerLim(3):res(3):robot.upperLim(3)]';

% Get all possible angle combinations
combs = combvec(th1s',th2s',th3s');

% Append two angles worth of zeros for theta4 and theta5 to fit the FK
% functions.
ths = [combs',zeros(size(combs,2),2)];
howMany = size(ths,1);

% Get the joint positions for all possible angle combinations
allJointPos = getJointPos(ths, robot);

% Define the beginning points for each link line
begPts = [squeeze(allJointPos(1,:,:))';
          squeeze(allJointPos(2,:,:))';
          squeeze(allJointPos(3,:,:))';
          squeeze(allJointPos(4,:,:))';
          squeeze(allJointPos(5,:,:))';
          squeeze(allJointPos(6,:,:))'];

% Define the end points for each link line
endPts = [squeeze(allJointPos(2,:,:))';
          squeeze(allJointPos(3,:,:))';
          squeeze(allJointPos(4,:,:))';
          squeeze(allJointPos(5,:,:))';
          squeeze(allJointPos(6,:,:))';
          squeeze(allJointPos(7,:,:))'];

% Initialize the free space as a vector row aligned with the angle combos
isFree = ones(howMany,1);

% For each obstacle in the space
for ii=1:size(map.obstacles)
    % Pad the obstacle
    currObstacle = [map.obstacles(ii,1:3)-mrgn,map.obstacles(ii,4:6)+mrgn];
  
    % Check for collisions with all the lines, which is a check for all
    % links of the robot.
    isCollidedAllLinks = detectCollision(begPts,endPts,currObstacle);
    
    % If any of the links are in collision, the whole configuration is in
    % collision
    isCollidedConfig = zeros(howMany,1);
    for jj = 1:6
        frstInd = howMany*(jj-1) + 1;
        lastInd = frstInd + howMany - 1;
        isCollidedConfig = isCollidedConfig | isCollidedAllLinks(frstInd:lastInd);
    end
    
    % The free C space is the set of configurations not in collision with
    % any of the obstacles.
    isFree = isFree & ~isCollidedConfig;
end

%% Create occupancy grid

% Construct voxel grid
cmap.occgrid = zeros(numel(th2s),numel(th1s),numel(th3s));

% Mark obstacles as occupied
occInds = pos2ind(cmap,ths(~isFree,1:3),1);
cmap.occgrid(occInds) = 1;

end

function J = getJointPos(q,robot)

d1 = robot.d1;
a2 = robot.a2;
a3 = robot.a3;
d4 = robot.d4;
d5 = robot.d5;
lg = robot.lg;

J = zeros(7,3,size(q,1));

t2 = cos(q(:,1));
t3 = sin(q(:,1));
t4 = pi./2.0;
t5 = -t4;
t6 = q(:,3)+t4;
t38 = t2.*6.123233995736766e-17;
t39 = t3.*6.123233995736766e-17;
t7 = q(:,2)+t5;
t8 = q(:,4)+t5;
t9 = cos(t6);
t10 = sin(t6);
t11 = cos(t7);
t12 = cos(t8);
t13 = sin(t7);
t14 = sin(t8);
t15 = a2.*t13;
t16 = t2.*t11;
t17 = t2.*t13;
t18 = t3.*t11;
t19 = t3.*t13;
t24 = t9.*t11;
t25 = t10.*t11;
t26 = t9.*t13;
t27 = t10.*t13;
t20 = a2.*t16;
t21 = a2.*t18;
t22 = -t15;
t23 = -t19;
t28 = a3.*t25;
t29 = a3.*t26;
t30 = -t27;
t33 = t25+t26;
t40 = t16.*6.123233995736766e-17;
t41 = t17.*6.123233995736766e-17;
t42 = t18.*6.123233995736766e-17;
t43 = t19.*6.123233995736766e-17;
t45 = t15.*t38;
t46 = t15.*t39;
t47 = t3.*t15.*(-6.123233995736766e-17);
t31 = -t28;
t32 = -t29;
t34 = t24+t30;
t35 = t14.*t33;
t44 = -t43;
t48 = t17+t42;
t49 = t18+t41;
t51 = t23+t40;
t36 = t12.*t34;
t50 = t16+t44;
t52 = t9.*t48;
t53 = t9.*t49;
t54 = t10.*t48;
t55 = t10.*t49;
t60 = t9.*t51;
t61 = t10.*t51;
t37 = -t36;
t56 = t9.*t50;
t57 = t10.*t50;
t58 = a3.*t53;
t59 = a3.*t54;
t62 = -t54;
t63 = -t55;
t65 = a3.*t61;
t68 = t53+t61;
t75 = -t12.*(t55-t60);
t64 = a3.*t56;
t66 = -t59;
t67 = t52+t57;
t69 = t56+t62;
t70 = t60+t63;
t72 = t14.*t68;
t73 = -t14.*(t54-t56);
t76 = t35+t37+3.749399456654644e-33;
t71 = t12.*t67;
t74 = -t72;
t77 = d5.*t76;
t82 = -d5.*(-t38+t72+t12.*(t55-t60));
t78 = t39+t71+t73;
t81 = t38+t74+t75;
t79 = d5.*t78;
t80 = -t79;

J(2,3,:) = d1;

J(3,1,:) = t20+t47;
J(3,2,:) = t21+t45;
J(3,3,:) = d1+t22;

J(4,1,:) = t20+t47+t64+t66;
J(4,2,:) = t21+t45+t58+t65;
J(4,3,:) = d1+t22+t31+t32;

J(5,1,:) = t3.*(-2.0818995585505e-15)+t20+t47+t64+t66-t71.*3.4e+1+t14.*(t54-t56).*3.4e+1;
J(5,2,:) = t2.*2.0818995585505e-15+t21+t45+t58+t65-t72.*d4-t12.*(t55-t60).*d4;
J(5,3,:) = d1+t22+t31+t32+t35.*d4-t36.*d4+1.274795815262579e-31;

J(6,1,:) = t20+t47+t64+t66+t80;
J(6,2,:) = t21+t45+t58+t65+t82;
J(6,3,:) = d1+t22+t31+t32+t77;

J(7,1,:) = t20+t47+t64+t66+t80-lg.*t78;
J(7,2,:) = t21+t45+t58+t65+t82-lg.*(-t38+t72+t12.*(t55-t60));
J(7,3,:) = d1+t22+t31+t32+t77+lg.*t76;

end

function pos = ind2pos(map,ind,order)
% IND2POS converts linear matrix indices to the [x y z] positions of the
% corresponding map voxel
%
% parameters:
%   map - map data structure loaded from a map textfile using load_map(...)
%   ind - nx1 vector of linear matrix indices
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%
% NOTE: this function assumes the given indices are within bounds of the
%       map matrix; bound checking must be performed before calling

[I,J,K] = ind2sub(size(map.occgrid), ind);
pos = sub2pos(map, [I(:) J(:) K(:)]);
end

function ind = pos2ind(map,pos,order)
% POS2IND converts [x y z] position vectors to the linear matrix indices of
% the corresponding map voxels
%
% parameters:
%   map - map data structure loaded from a map textfile using load_map(...)
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%   ind - nx1 vector of linear matrix indices
%
% NOTE: this function assumes the given points are within bounds of the
%       map matrix; bound checking must be performed before calling

ijk = pos2sub(map, pos);
ind = sub2ind(size(map.occgrid),ijk(:,1), ijk(:,2), ijk(:,3));
end

function [ijk] = pos2sub(map, pos, order)
% POS2SUB converts [x y z] position vectors into the subscripts [i, j, k]
% of the corresponding map voxels
%
% parameters:
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%   map - map data structure loaded from a map textfile using load_map(...)
%   ijk - nx3 matrix of subscripts; each row is a set of subscripts
%         describing a voxel in the map
%
% NOTE: this function assumes pos is inside the map; bound checking must
%       be performed before calling

[jik] = round((pos - map.bound_th123(1:3))./map.res_th123) + 1;
ijk = [jik(:,2) jik(:,1) jik(:,3)];
end

function [pos] = sub2pos(map, ijk)
% SUB2POS converts an [i j k] vector of subscripts to the world [x y z]
% position of the corresponding map voxel
%
% parameters:
%   ijk - nx3 matrix of subscripts; each row is a set of subscripts
%         describing a voxel in the map
%   map - map data structure loaded from a map textfile using load_map(...)
%   pos - nx3 matrix of points; each row is an [x y z] position vector
%
% NOTE: this function assumes the give subscripts are within bounds of the
%       map matrix; bound checking must be performed before calling

pos = ([ijk(:,2) ijk(:,1) ijk(:,3)] - 1).*map.res_th123 + map.bound_th123(1:3);
end