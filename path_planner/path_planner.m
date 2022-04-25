%% Example code to run trajectory using variables created by SimSpace.m
% run SimSpace.m
% 
% %%
% bias = sqrt(xGoal^2 + yGoal^2); % Suggested to have Bias < 1
% MaxDist = 0.01; % Suggested to have MaxDist < 0.05
% MaxTree = 2000;
% enable_visual = 1;
% 
% safety = 2; % Suggested to have safety radius factor of > 1.5
% 
% obstacles(1).xv = obs1X + safety*obs1R*cos(linspace(0,2*pi,20))';
% obstacles(1).yv = obs1Y + safety*obs2R*sin(linspace(0,2*pi,20))';
% 
% obstacles(2).xv = obs2X + safety*obs1R*cos(linspace(0,2*pi,20))';
% obstacles(2).yv = obs2Y + safety*obs2R*sin(linspace(0,2*pi,20))';
% 
% start = [xStart yStart];
% goal = [xGoal yGoal];
% 
% points = test(start, goal, obstacles, bias, MaxDist, MaxTree, enable_visual);

%% Function
function [path] = path_planner(start, goal, obstacles, bias, MaxDist, MaxTree, enable_visual)
    % Perform RRTstar path planning
    [~, ~, waypoints] = RRTstar(bias, MaxDist, MaxTree, enable_visual, -0.45, 0, 0.45, 0.45, start(1), start(2), goal(1), goal(2), obstacles);

    domain = 1:1:length(waypoints(1,:)); % Initial domain
    domain_samp = 1:0.01:length(waypoints(1,:)); % 100x sampling domain
    x_samp = smooth(interp1(domain,waypoints(1,:),domain_samp,'spline')); % interpolate and smooth domain
    y_samp = smooth(interp1(domain,waypoints(2,:),domain_samp,'spline')); % interpolate and smooth range
    plot(x_samp, y_samp, 'b', 'Linewidth', 1.5) % plot smoothed XY path over result
    
    path.x = x_samp;
    path.y = y_samp;
end