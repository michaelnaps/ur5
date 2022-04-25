%% ME 5463 - Final Project
% Group Members: Michael Napoli, Alexander Ventura,
%   Dylan Trainor, Jesse Zhang, Chunhui Wang

restoredefaultpath;

clc;clear;
close all;

addpath /home/michaelnaps/prog/ModernRobotics/packages/MATLAB/mr
addpath ./controller
addpath ./general
addpath ./path_planner
addpath ./ur5_system
addpath ./run_data

%% Load UR5 Data and Compute Values
[home, joints, links] = home_ur5("ur5.urdf");

M = home.M;
Tlist = home.Tlist;
Mlist = home.Mlist;
Glist = home.Glist;
Slist = home.Slist;

%% calculate desired path
run SimSpace.m
obstacles = env.obstacles;

%%
bias1 = 2.0; % Suggested to have Bias < 1
bias2 = 1.0;
MaxDist = 0.01; % Suggested to have MaxDist < 0.05
MaxTree = 2000;
enable_visual = 1;

safety = 2; % Suggested to have safety radius factor of > 1.5

obstacles(1).xv = obs1X + safety*obs1R*cos(linspace(0,2*pi,20))';
obstacles(1).yv = obs1Y + safety*obs2R*sin(linspace(0,2*pi,20))';

obstacles(2).xv = obs2X + safety*obs1R*cos(linspace(0,2*pi,20))';
obstacles(2).yv = obs2Y + safety*obs2R*sin(linspace(0,2*pi,20))';

xObject = mean(mean(objects.x));
yObject = mean(mean(objects.y));

start  = [xStart yStart];
object = [xObject yObject];
goal   = [env.Goal.X env.Goal.Y];

fprintf("Path Planner: object->start\n")
tic
start2obj = path_planner(object, start, obstacles, bias1, MaxDist, MaxTree, enable_visual);
N_start2obj = length(start2obj.x);
toc

fprintf("\nPath Planner: goal->object\n")
tic
obj2goal  = path_planner(goal, object, obstacles, bias2, MaxDist, MaxTree, enable_visual);
N_obj2goal = length(obj2goal.x);
toc

pause(5);
%% variables for config functions
close all;
R = 0.5;
H1 = 0.01;  H2 = 0.2;

z1 = H1*ones(N_start2obj,1);
z2 = H2*ones(N_obj2goal,1);

Nadj = 500;
x1adj = linspace(start2obj.x(end), start2obj.x(end), Nadj)';
y1adj = linspace(start2obj.y(end), start2obj.y(end), Nadj)';
z1adj = linspace(H1,H2,Nadj)';

x2adj = linspace(obj2goal.x(end), obj2goal.x(end), Nadj)';
y2adj = linspace(obj2goal.y(end), obj2goal.y(end), Nadj)';
z2adj = linspace(H2,H1,Nadj)';

path.x = [start2obj.x; x1adj; obj2goal.x; x2adj];
path.y = [start2obj.y; y1adj; obj2goal.y; y2adj];
path.z = [z1; z1adj; z2; z2adj];
% path.ee = [zeros(N_start2obj,1), ones(N_obj2goal,1)];

N = length(path.x);

path.R = ones(3,3,N).*rotMat('x',pi);

t = linspace(0, 30, N);

fprintf("\nJoint Configurations:\n")
tic
q_d = pathconfig_ur5(home, path);
q_d = q_d';
toc

%% setup input variable and enter PID
q0 = [q_d(:,1); zeros(6,1)];

inputs.home = home;
inputs.NF = 2;
inputs.DR = 1;
inputs.dt = t(2) - t(1);

%% finite difference for dqd and ddqd
dt    = inputs.dt;
dq_d  = fdm(q_d, dt);
ddq_d = fdm(dq_d, dt);

inputs.q_d   = q_d;
inputs.dq_d  = dq_d;
inputs.ddq_d = ddq_d;

fprintf("\nPath Follow Controller\n")
tic
[t, q] = ode45(@(t,q) trajectory_ur5(t,q,inputs), t, q0);
toc

%% results
% load("ws_fullRun03.mat")
animation_ur5(t, q, joints, Slist, path, env);
pause(5); close all;

%% plotting
figure(1)
subplot(3,1,1)
plot(t, q_d)
title("Desired Joint Positions")
ylabel("Joints Pos [rad]")
xlabel("Time [s]")
legend("Link 1", "Link 2", "Link 3", "Link 4", "Link 5", "Link 6")
subplot(3,1,2)
plot(t, dq_d)
title("Desired Joint Velocity")
ylabel("Joints Pos [rad/s]")
xlabel("Time [s]")
subplot(3,1,3)
plot(t, ddq_d)
title("Desired Joint Acceleration")
ylabel("Joints Acc. [rad/s^2]")
xlabel("Time [s]")

%% accuracy plots
for i = 1:6
    figure(4)
    subplot(2,3,i)
    plot(t, q_d(i,:), '--r'); hold on
    plot(t, q(:,i), 'color', '#77AC30')
    title("Link " + i)
    sgtitle("Desired vs. Actual Joint Positions")
    ylabel("Position [rad]")
    xlabel("Time [s]")
    legend("Desired", "Actual")

    figure(5)
    subplot(2,3,i)
    plot(t, dq_d(i,:), '--r'); hold on
    plot(t, q(:,i+6), 'color', '#77AC30')
    title("Link " + i)
    sgtitle("Desired vs. Actual Joint Velocities")
    ylabel("Position [rad]")
    xlabel("Time [s]")
    legend("Desired", "Actual")

    figure(6)
    subplot(2,3,i)
    plot([t(1), t(end)], [0, 0], '--r'); hold on
    plot(t, q(:,i)-q_d(i,:)', 'color', '#77AC30'); hold on
    plot(t, q(:,i+6)-dq_d(i,:)', 'color', '#0072BD')
    title("Link " + i)
    sgtitle("Error for Position and Velocity Behavior")
    ylabel("Error")
    xlabel("Time [s]")
    legend("", "Pos", "Vel")
end













