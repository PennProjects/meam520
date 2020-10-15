function [isCollided] = detectCollision(linePt1, linePt2, box)
% DETECTCOLLISION This function detects collisions between a line
%   (representing one link of the Lynx robot) and an obstacle defined as an
%   axis-aligned bounding box. It can be used for any number of lines, but \
%   only one box per call.
%
% INPUTS
%   linePt1 - an Nx3 array where each row represents (x,y,z) coordinates of
%               the beginning of the line being tested, row aligned with
%               linePt2.
%   linePt2 - an Nx3 array where each row represents (x,y,z) coordinates of
%               the end of the line being tested, row aligned with linePt1.
%   box     - a 1x6 or 6x1 vector of coordinates defining an axis-aligned
%               bounding box. Thus (x1,y1,z1,x2,y2,z2).
%
% OUTPUTS
%   isCollided - an Nx1 vector of boolean flags, where 1 corresponds to a
%                   collision and 0 corresponds to no collision betweened
%                   the box and the line that is composed of the
%                   row-aligned rows of linePt1 and linePt2.
%
% ASSUMPTIONS
%   (1) - Line is infinitely thin (students may have to draw extra space
%           around the bounding box to account for this).
%   (2) - If any point on the box is contained inside the open set of the
%           box, there is an intersection. Even an intersection at one
%           point will cause a collision.
%   (3) - The points chosen are the endpoints of the line, in no particular
%           order. The line does not extend past these endpoints in either
%           direction.
%   (4) - The boxSize values are all positive, and correspond to x, y,
%           and z dimensions respectively.
%
% AUTHOR:
%   Jaimie Carlson, September 30, 2018 - initial function
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu) - modified for
%           vectorization to process multiple lines at once.
%
% REFERENCE:
%   https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

%% Initialization

% Initialize all lines as collided.
isCollided = ones(size(linePt1,1),1);

% Divide box into lower left point and "size"
boxPt1 = box(1:3);
boxSize = box(4:6) - boxPt1;
% Create point in the opposite corner of the box
boxPt2 = boxPt1 + boxSize;

% Find slopes
lineSlope = linePt2 - linePt1;

%% Begin Collision Detection
%
% Collision detection is based on identifying whether the ray passes
% through the box in each of the planes. An intersection occurs ONLY if the
% ray passes through the box in all three planes.

% The parameter t = [0,1] traces out from linePt1 to linePt2.

% Return false if box is invalid or has a 0 dimension
if (min(boxSize) <= 0)
    isCollided = 0*isCollided;
    return;
end

% Get minimum and maximum intersection with the y-z planes of the box
txmin = (boxPt1(1) - linePt1(:,1))./lineSlope(:,1);
txmax = (boxPt2(1) - linePt1(:,1))./lineSlope(:,1);
% Put them in order based on the parameter t.
ts = sort([txmin,txmax],2);
txmin = ts(:,1); txmax = ts(:,2);

% Get minimum and maximum intersection with the x-z planes of the box
tymin = (boxPt1(2) - linePt1(:,2))./lineSlope(:,2);
tymax = (boxPt2(2) - linePt1(:,2))./lineSlope(:,2);
% Put them in order based on the parameter t.
ts = sort([tymin,tymax],2);
tymin = ts(:,1); tymax = ts(:,2);

% If we miss the box in this plane, no collision
isCollided = isCollided & ~((txmin > tymax) | (tymin > txmax));

%Identify the parameters to use with z
tmin = max(txmin, tymin);
tmax = min(txmax, tymax);


% Get minimum and maximum intersection with the x-y planes of the box
tzmin = (boxPt1(3) - linePt1(:,3))./lineSlope(:,3);
tzmax = (boxPt2(3) - linePt1(:,3))./lineSlope(:,3);
% Put them in order based on the parameter t.
ts = sort([tzmin,tzmax],2);
tzmin = ts(:,1); tzmax = ts(:,2);

% If we miss the box in this plane, no collision
isCollided = isCollided & ~((tmin > tzmax) | (tzmin > tmax));



%Identify the parameters to use for place check
tmin = max(tmin, tzmin);
tmax = min(tmax, tzmax);

% Check that the intersection is within the link length
isCollided = isCollided & ~(0 > tmax | 1 < tmin);

end
