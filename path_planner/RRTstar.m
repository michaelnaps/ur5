% This function is modifed based on the example developed by 
% Omkar Halbe, Technical University of Munich, 31.10.2015
function [cost,iter,waypoints] = RRTstar(bias, MaxDist, MaxTree, enable_visual, xMin, yMin, xMax, yMax, xStart, yStart, xGoal, yGoal, obstacles)
% clear; clc
% close all;

%% initialization

nbh = 0.2;    % neighborhood region
thresh = 0.025;   % acceptable distance from the goal point


% we will use an structure array to represent the tree structure
% each entry in the array represent one node in the tree 
% (i.e., one point in the 2D configuration space)
% each node the following element:
%  - x: the x coordinate of the node
%  - y: the y coordinate of the node
%  - cost: the cost to arrive at this node
%  - parent: the index of the parent node 

% The tree will be initialized by the starting point
nodes(1).x = xStart;
nodes(1).y = yStart;
nodes(1).cost = 0;
nodes(1).parent = 0;

%% visualization
if enable_visual
    figure(1); hold on; grid on
    
    % axis equal
    % axis([0,100,0,100]);
    % plot the boundary
    % plot([0,0,100,100,0],[0,100,100,0,0],'k--','LineWidth',1);
    % plot the starting point
    plot(xStart, yStart, 'ko', 'MarkerSize',3, 'MarkerFaceColor','k');
    % plot the goal point and goal region
    plot(xGoal, yGoal, 'go', 'MarkerSize',3, 'MarkerFaceColor','g');
    th = 0:pi/50:2*pi;
    xcircle = thresh * cos(th) + xGoal;
    ycircle = thresh * sin(th) + yGoal;
    h = plot(xcircle, ycircle);
    % plot the obstacles
    for i=1:length(obstacles)
        plot(obstacles(i).xv,obstacles(i).yv,'k','LineWidth',1);
    end
end


%% search
iter = 2;
% the value of "bias" can affect how to sample from the C-space
% bias = 1: no bias
% bias < 1: biased toward the goal  (xMax, yMax)
% bias > 1: biased toward the start (xMin, yMin)
% bias = 1;%1/sqrt(2);
while iter < MaxTree   
    % sample a random point
    [xRand, yRand] = random_sample(xMin,yMin,xMax,yMax,bias);
    % sort the tree to find the nearest node
    p_index = sort_tree(nodes, xRand, yRand);
    xNear = nodes(p_index).x;
    yNear = nodes(p_index).y;
    % plan to motion from the nearest point to the random point by MaxDist
    [xNew, yNew] = local_planner(xNear,yNear,xRand,yRand,MaxDist);
    % check collision of the new path from the nearest point to the new
    % point
    c_test = collision_detector(xNear,yNear,xNew, yNew,obstacles);
    % if there is a collision, dump the current trial
    if c_test
        continue;
    end
    
    % update the tree    
    nodes(iter).x = xNew;       %#ok<*AGROW,*SAGROW> 
    nodes(iter).y = yNew;       % the new point as a new node
    
    node_cost = distance(xNew,yNew,xNear,yNear) + nodes(p_index).cost; %parent's cost + distance
    for j=1:iter-1
        x_tmp = nodes(j).x;
        y_tmp = nodes(j).y;
        c_test = collision_detector(x_tmp,y_tmp,xNew, yNew,obstacles);
        if c_test
            continue;
        end
        tmp_dist = distance(xNew,yNew,x_tmp,y_tmp);
        if  tmp_dist <= nbh
            tmp_cost = tmp_dist + nodes(j).cost;
            if tmp_cost < node_cost
                node_cost = tmp_cost;
                p_index = j;
            end
        end
        
    end
    
    
    nodes(iter).parent = p_index; % the nearest point is the parent node
    nodes(iter).cost = node_cost;
    
    % plot the new path
    if enable_visual
        plot([nodes(iter).x; nodes(p_index).x],[nodes(iter).y; nodes(p_index).y], 'r');
        pause(0.01);
    end
    
    % check if it reaches the goal region
    if distance(xNew, yNew, xGoal, yGoal) <= thresh
        break
    end
    
    iter = iter + 1;
end

%% Replot
for i = 2:length(nodes)
    % plot the new path
    if enable_visual
        plot([nodes(i).x; nodes(nodes(i).parent).x],[nodes(i).y; nodes(nodes(i).parent).y], 'r');
        pause(0.001);
    end
end


%% final cost
cost = nodes(end).cost;

%% determine the feasible path

if iter < MaxTree
    xPath(1) = xGoal;
    yPath(1) = yGoal;
    xPath(2) = nodes(end).x;
    yPath(2) = nodes(end).y;
    
    parent = nodes(end).parent;
    j=0;
    while 1
        xPath(j+3) = nodes(parent).x;
        yPath(j+3) = nodes(parent).y;        
        parent = nodes(parent).parent;
        if parent == 0
            break
        end
        j=j+1;
    end
    
    waypoints = [xPath;yPath];
    
    if enable_visual
        plot(xPath, yPath, 'g', 'Linewidth', 3);
    end
else
    disp('No path found. Increase number of iterations and retry.');
    cost = NaN;
    iter = NaN;
end