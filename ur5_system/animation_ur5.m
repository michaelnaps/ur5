function disp = animation_ur5(t, x, joints, Slist, path, env, speed_factor)

if nargin < 7
    speed_factor = 1;  % >1 speed up, <1 slow down
end


t1 = 0:0.01:t(end);
x1 = interp1(t,x,t1);
n = length(t1);
p0 = zeros(3,n); % base origin, it remains at zero
% initialize vectrs
p1 = zeros(3,n); % joint1 origin
p2 = zeros(3,n); % joint2 origin
p3 = zeros(3,n); % joint3 origin
p4 = zeros(3,n); % joint4 origin
p5 = zeros(3,n); % joint5 origin
p6 = zeros(3,n); % joint6 origin
p_ee = zeros(3,n); % end effector frame origin
p_ee_x = zeros(3,n); % end effector frame x-axis
p_ee_y = zeros(3,n); % end effector frame y-axis
p_ee_z = zeros(3,n); % end effector frame z-axis
q = x1(:,1:6)';% joint position variables, convert to column variables
for i=1:n
    p1(:,i) = joints(2).M(1:3,4); % it remains constant, joint 1 does not change its location
    T2 = FKinSpace(joints(3).M, Slist(:,1),q(1,i)); % joint2 frames changes only due to joint1 motion
    p2(:,i) = T2(1:3,4); % extract the position
    T3 = FKinSpace(joints(4).M, Slist(:,1:2),q(1:2,i)); % joint2 frames changes only due to joint1 and joint2 motions
    p3(:,i) = T3(1:3,4); % extract the position
    T4 = FKinSpace(joints(5).M, Slist(:,1:3),q(1:3,i)); 
    p4(:,i) = T4(1:3,4); % extract the position
    T5 = FKinSpace(joints(6).M, Slist(:,1:4),q(1:4,i)); 
    p5(:,i) = T5(1:3,4); % extract the position
    T6 = FKinSpace(joints(7).M, Slist(:,1:5),q(1:5,i)); 
    p6(:,i) = T6(1:3,4); % extract the position
    T_ee = FKinSpace(joints(8).M, Slist(:,1:6),q(1:6,i)); 
    p_ee(:,i) = T_ee(1:3,4); % extract the position
    p_ee_x(:,i) = 0.05*T_ee(1:3,1) + T_ee(1:3,4);
    p_ee_y(:,i) = 0.05*T_ee(1:3,2) + T_ee(1:3,4);
    p_ee_z(:,i) = 0.05*T_ee(1:3,3) + T_ee(1:3,4);
end

disp = figure;clf;
disp.Position = [400 200 720 720];
hold on;

% axis off;
% plot([-0.05,0.05],[0,0],'k-','LineWidth',2)

base_link = plot3([p0(1,1),p1(1,1)],[p0(2,1),p1(2,1)],[p0(3,1),p1(3,1)],'Color',[0 0.4470 0.7410],'LineWidth',3);
link1 = plot3([p1(1,1),p2(1,1)],[p1(2,1),p2(2,1)],[p1(3,1),p2(3,1)],'Color',[0.8500 0.3250 0.0980],'LineWidth',3);
link2 = plot3([p2(1,1),p3(1,1)],[p2(2,1),p3(2,1)],[p2(3,1),p3(3,1)],'Color',[0.9290 0.6940 0.1250],'LineWidth',3);
link3 = plot3([p3(1,1),p4(1,1)],[p3(2,1),p4(2,1)],[p3(3,1),p4(3,1)],'Color',[0.4940 0.1840 0.5560],'LineWidth',3);
link4 = plot3([p4(1,1),p5(1,1)],[p4(2,1),p5(2,1)],[p4(3,1),p5(3,1)],'Color',[0.4660 0.6740 0.1880],'LineWidth',3);
link5 = plot3([p5(1,1),p6(1,1)],[p5(2,1),p6(2,1)],[p5(3,1),p6(3,1)],'Color',[0.3010 0.7450 0.9330],'LineWidth',3);
link6 = plot3([p6(1,1),p_ee(1,1)],[p6(2,1),p_ee(2,1)],[p6(3,1),p_ee(3,1)],'Color',[0.6350 0.0780 0.1840],'LineWidth',3);
ee_x_axis = plot3([p_ee(1,1),p_ee_x(1,1)],[p_ee(2,1),p_ee_x(2,1)],[p_ee(3,1),p_ee_x(3,1)],'Color','k','LineWidth',1);
ee_y_axis = plot3([p_ee(1,1),p_ee_x(1,1)],[p_ee(2,1),p_ee_x(2,1)],[p_ee(3,1),p_ee_x(3,1)],'Color','k','LineWidth',1);
ee_z_axis = plot3([p_ee(1,1),p_ee_x(1,1)],[p_ee(2,1),p_ee_x(2,1)],[p_ee(3,1),p_ee_x(3,1)],'Color','k','LineWidth',1);

xObject = mean(mean(env.objects.x));
yObject = mean(mean(env.objects.y));

object = plot3(xObject, yObject, 0.01, '.k', 'markersize', 1500*env.objects.R);

%% plot the environment

% plot the initial region 
patch([env.initial(1).X,env.initial(2).X,env.initial(3).X,env.initial(4).X], ...
    [env.initial(1).Y,env.initial(2).Y,env.initial(3).Y,env.initial(4).Y],[0,0,0,0],'y');
% plot the obstacle region
patch([env.obs(1).X,env.obs(2).X,env.obs(3).X,env.obs(4).X], ...
    [env.obs(1).Y,env.obs(2).Y,env.obs(3).Y,env.obs(4).Y],[0,0,0,0],'blue');
% plot the goal table
patch([env.tar(1).X,env.tar(2).X,env.tar(3).X,env.tar(4).X], ...
    [env.tar(1).Y,env.tar(2).Y,env.tar(3).Y,env.tar(4).Y],[0,0,0,0],'c');
% plot the goal point and goal region
plot3(env.Goal.X, env.Goal.Y, env.Goal.Z, 'go', 'MarkerSize',3, 'MarkerFaceColor','r');
th = 0:pi/50:2*pi;
thresh = 0.025;
xcircle = thresh * cos(th) + env.Goal.X;
ycircle = thresh * sin(th) + env.Goal.Y;
plot(xcircle, ycircle);
a1 = linspace(0,pi,30);
a2 = linspace(0,2*pi,30);
[th,ph] = meshgrid(a1,a2);
% plot the obstacles
for i=1:length(env.obstacles)
surf(env.obstacles(i).x, env.obstacles(i).y, env.obstacles(i).z);
colormap([0 1 0]);
hold on
fill3(env.obstacle(i).X,env.obstacle(i).Y,zeros(20,1),'g');
hold on
fill3(env.obstacle(i).X,env.obstacle(i).Y,env.obsH(i)*ones(20,1),'g');
end
%% goal path
pathPlot = plot3(path.x, path.y, path.z);

ee_traj = animatedline(p_ee(1,1),p_ee(2,1),p_ee(3,1),'Color','#d1e0e0','LineWidth',2);

xlabel('x (m)','FontSize',14);
ylabel('y (m)','FontSize',14);
zlabel('z (m)','FontSize',14);

% axis manual;
view(25,15);
axis([-0.6 0.6 -0.2 0.6 -0.1 0.6]);
axis equal
grid on
deltaT = 0.005/speed_factor;

pickup = 0;
for i=1:n
    % base link
    base_link.XData = [p0(1,i),p1(1,i)];
    base_link.YData = [p0(2,i),p1(2,i)];
    base_link.ZData = [p0(3,i),p1(3,i)];
    % link1
    link1.XData = [p1(1,i),p2(1,i)];
    link1.YData = [p1(2,i),p2(2,i)];
    link1.ZData = [p1(3,i),p2(3,i)];
    % link2
    link2.XData = [p2(1,i),p3(1,i)];
    link2.YData = [p2(2,i),p3(2,i)];
    link2.ZData = [p2(3,i),p3(3,i)];
    % link3
    link3.XData = [p3(1,i),p4(1,i)];
    link3.YData = [p3(2,i),p4(2,i)];
    link3.ZData = [p3(3,i),p4(3,i)];
    % link4
    link4.XData = [p4(1,i),p5(1,i)];
    link4.YData = [p4(2,i),p5(2,i)];
    link4.ZData = [p4(3,i),p5(3,i)];
    % link5
    link5.XData = [p5(1,i),p6(1,i)];
    link5.YData = [p5(2,i),p6(2,i)];
    link5.ZData = [p5(3,i),p6(3,i)];
    % link6
    link6.XData = [p6(1,i),p_ee(1,i)];
    link6.YData = [p6(2,i),p_ee(2,i)];
    link6.ZData = [p6(3,i),p_ee(3,i)];
    % end effector frame x-axis
    ee_x_axis.XData = [p_ee(1,i),p_ee_x(1,i)];
    ee_x_axis.YData = [p_ee(2,i),p_ee_x(2,i)];
    ee_x_axis.ZData = [p_ee(3,i),p_ee_x(3,i)];
    % end effector frame y-axis
    ee_y_axis.XData = [p_ee(1,i),p_ee_y(1,i)];
    ee_y_axis.YData = [p_ee(2,i),p_ee_y(2,i)];
    ee_y_axis.ZData = [p_ee(3,i),p_ee_y(3,i)];
    % end effector frame z-axis
    ee_z_axis.XData = [p_ee(1,i),p_ee_z(1,i)];
    ee_z_axis.YData = [p_ee(2,i),p_ee_z(2,i)];
    ee_z_axis.ZData = [p_ee(3,i),p_ee_z(3,i)];

    % object location
    if ((p_ee(1,i)-xObject) < 0.01 && (p_ee(2,i)-yObject) < 0.01) || pickup
        object.XData = p_ee(1,i);
        object.YData = p_ee(2,i);
        object.ZData = p_ee(3,i);
        pickup = 1;
    end

    drawnow;    
    pause(deltaT);
end
end

