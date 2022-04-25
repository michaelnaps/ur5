%% Construction of 2D simulation space

% clear; clc
% close all;

%% Initialization

% initial map size: (xMax * yMax * zMax)

xMin = -0.6;
yMin = -0.5;
zMin = 0;
xMax = 0.6;
yMax = 1;
zMax = 1.2;

% initial position
xStart = -0.25;
yStart = 0.025;
zStart = 0;

% location of base joint
xBase = 0;
yBase = 0;
zBase = 0;


%% Initial region area

% Initial rectangular region four vertices coordinates: (x,y) = (initial(i).X, initial(i).Y)

initial(1).X = -0.45;
initial(2).X = -0.225;
initial(3).X = -0.225;
initial(4).X = -0.45;

initial(1).Y = 0;
initial(2).Y = 0;
initial(3).Y = 0.45;
initial(4).Y = 0.45;

% obstacle boundary
obs(1).X = initial(2).X;
obs(2).X = 0.225;
obs(3).X = 0.225;
obs(4).X = initial(3).X;

obs(1).Y = 0.1;
obs(2).Y = 0.1;
obs(3).Y = 0.45;
obs(4).Y = 0.45;
% goal table (target region)
tar(1).X = obs(2).X;
tar(2).X = 0.45;
tar(3).X = 0.45;
tar(4).X = obs(3).X;

tar(1).Y = 0;
tar(2).Y = 0;
tar(3).Y = 0.45;
tar(4).Y = 0.45;
% goal position
Goal.X = tar(1).X+(tar(2).X-tar(1).X)*rand;
Goal.Y = tar(2).Y+(tar(3).Y-tar(1).Y)*rand;
Goal.Z = 0;
thresh = 0.025;
%% Obstacles

% location and radius of the four obstacles are:
% obstacle(i): 
% location: (X,Y)=(obsX,obsY); 
% radius: R=obsR;

% centerpoint location of obstacles and radius

% radius of the cylinder
obs1R = 0.05;
obs1X = obs(1).X+obs1R + (obs(2).X-2*obs1R-obs(1).X).*rand;     % x location
obs1Y = obs(2).Y+obs1R+(obs(3).Y-2*obs1R-obs(2).Y).*rand;     % y location
     
obsH = zeros(1,4);
obsH(1) = rand*0.2;     % height of the cylinder

obs2R = 0.06;
obs2X = obs(1).X+obs2R + (obs(2).X-2*obs2R-obs(1).X).*rand;
obs2Y = obs(2).Y+obs2R+(obs(3).Y-obs2R-2*obs(2).Y).*rand; 

obsH(2) = rand*0.2;

%obs3X = 1.2;
%obs3Y = 0.5;
%obs3R = 0.15;
%obsH(3) = 0.2;

%obs4X = 1.2;
%obs4Y = 1.2;
%obs4R = 0.15;
%obsH(4) = 0.15;

% obstacles in 3D
obstacle(1).X = obs1X+obs1R*cos(linspace(0,2*pi,20))';
obstacle(1).Y = obs1Y+obs1R*sin(linspace(0,2*pi,20))';
obstacle(1).Z = linspace(0,obsH(1),20);

obstacles(1).x = repmat(obstacle(1).X,1,20);
obstacles(1).y = repmat(obstacle(1).Y,1,20);
obstacles(1).z = repmat(obstacle(1).Z,20,1);

obstacle(2).X = obs2X+obs2R*cos(linspace(0,2*pi,20))';
obstacle(2).Y = obs2Y+obs2R*sin(linspace(0,2*pi,20))';
obstacle(2).Z = linspace(0,obsH(2),20);

obstacles(2).x = repmat(obstacle(2).X,1,20);
obstacles(2).y = repmat(obstacle(2).Y,1,20);
obstacles(2).z = repmat(obstacle(2).Z,20,1);

%obstacle(3).X = obs3X+obs3R*cos(linspace(0,2*pi,20))';
%obstacle(3).Y = obs3Y+obs3R*sin(linspace(0,2*pi,20))';
%obstacle(3).Z = linspace(0,obsH(3),20);

%obstacles(3).x = repmat(obstacle(3).X,1,20);
%obstacles(3).y = repmat(obstacle(3).Y,1,20);
%obstacles(3).z = repmat(obstacle(3).Z,20,1);

%obstacle(4).X = obs4X+obs4R*cos(linspace(0,2*pi,20))';
%obstacle(4).Y = obs4Y+obs4R*sin(linspace(0,2*pi,20))';
%obstacle(4).Z = linspace(0,obsH(4),20);

%obstacles(4).x = repmat(obstacle(4).X,1,20);
%obstacles(4).y = repmat(obstacle(4).Y,1,20);
%obstacles(4).z = repmat(obstacle(4).Z,20,1);

%% Objects 

% object location and radius are:
% object(i): 
% location (X,Y)=(obj(i).X, obj(i).Y); 
% radius = objR;

% define object locations
objects(1).R = 0.025; % radius of sphere

% nodes plane
objTh = linspace(0,pi,30);
%obj(2).X = objR(2)*cos(objTh);
%obj(2).Y = objR(2)*sin(objTh);

% 3D sphere
obj_x = initial(1).X+objects(1).R+(initial(2).X-2*objects(1).R-initial(1).X).*rand;
obj_y = initial(2).Y+objects(1).R+(initial(3).Y-2*objects(1).R-initial(2).Y).*rand;

objTh1 = linspace(0,2*pi,30)';
objects(1).x = obj_x+objects(1).R*sin(objTh).*cos(objTh1);
objects(1).y = obj_y+objects(1).R*sin(objTh).*sin(objTh1);

% plot the objects
%for i=1:length(objects)
%    surf(objects(i).x,objects(i).y,objects(i).R+repmat(obj(i).X,50,1));
%end

%objects(2).x = 0.4+obj(2).Y.*cos(objTh1);
%objects(2).y = 1+obj(2).Y.*sin(objTh1);

%% Visualization (note)
%{
figure(); hold on; grid on;
xlabel('x')
ylabel('y')
zlabel('z')

axis equal
axis([0,xMax,0,yMax,0,zMax]);
% plot the boundary
plot3([xMin,xMax,xMax,xMin,xMin,xMin,xMax,xMax,xMax,xMax,xMax,xMin,xMin,xMax,xMin,xMin,xMax,xMax],...
    [yMin,yMin,yMax,yMax,yMin,yMin,yMin,yMin,yMin,yMax,yMax,yMax,yMax,yMax,yMax,yMin,yMin,yMin],...
    [zMin,zMin,zMin,zMin,zMin,zMax,zMax,zMax,zMax,zMax,zMin,zMin,zMax,zMax,zMax,zMax,zMax,zMin],'k--','LineWidth',1);
% plot the starting point
plot(xStart, yStart, 'ko', 'MarkerSize',3, 'MarkerFaceColor','k');

plot(xBase, yBase,'ko', 'MarkerSize',5, 'MarkerFaceColor','g');

% plot the initial region 
patch([initial(1).X,initial(2).X,initial(3).X,initial(4).X], ...
    [initial(1).Y,initial(2).Y,initial(3).Y,initial(4).Y],[0,0,0,0],'y');
% plot the obstacle region
patch([obs(1).X,obs(2).X,obs(3).X,obs(4).X], ...
    [obs(1).Y,obs(2).Y,obs(3).Y,obs(4).Y],[0,0,0,0],'blue');
% plot the goal table
patch([tar(1).X,tar(2).X,tar(3).X,tar(4).X], ...
    [tar(1).Y,tar(2).Y,tar(3).Y,tar(4).Y],[0,0,0,0],'c');
% plot the goal point and goal region
plot3(Goal.X, Goal.Y, Goal.Z, 'go', 'MarkerSize',3, 'MarkerFaceColor','r');
th = 0:pi/50:2*pi;
xcircle = thresh * cos(th) + Goal.X;
ycircle = thresh * sin(th) + Goal.Y;
h = plot(xcircle, ycircle);
% plot the obstacles
for i=1:length(obstacles)
surf(obstacles(i).x, obstacles(i).y, obstacles(i).z);
colormap([0 1 0]);
hold on
fill3(obstacle(i).X,obstacle(i).Y,zeros(20,1),'g');
hold on
fill3(obstacle(i).X,obstacle(i).Y,obsH(i)*ones(20,1),'g');
end

% plot the objects
for i=1:length(objects)
    surf(objects(i).x,objects(i).y,objects(i).R+repmat(objects(1).R*cos(objTh),30,1));
end
%}
%ouput env. params
env.initial = initial;
env.obs = obs;
env.tar = tar;
env.Goal = Goal;
env.obstacles = obstacles;
env.objects = objects;
env.obstacle =obstacle;
env.obsH = obsH;