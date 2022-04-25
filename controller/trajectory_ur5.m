function [ddq] = trajectory_ur5(t, q0, inputs)
    %% Constant Matrices
    Mlist = inputs.home.Mlist;
    Glist = inputs.home.Glist;
    Slist = inputs.home.Slist;

    %% Current State Variable Setup
    N = length(q0)/2;
    q = q0(1:N);  dq = q0(N+1:end);
    g = [0, 0, -9.81]';
    
    % system force components
    M = MassMatrix(q, Mlist, Glist, Slist);
    C = VelQuadraticForces(q, dq, Mlist, Glist, Slist);
    G = GravityForces(q, g, Mlist, Glist, Slist);

    %% PD Controller
    dt = inputs.dt;
    index = round(t/dt)+1;

    % fdm function for dq_d and ddq_d
    % target position
    q_d   = inputs.q_d(:,index);
    dq_d  = inputs.dq_d(:,index);
    ddq_d = inputs.ddq_d(:,index);

    % desired mass matrix
    Md = MassMatrix(q_d, Mlist, Glist, Slist);
    
    % natural frequency and damping ratio
    NF = inputs.NF;
    DR = inputs.DR;

    % PD controller gains
    kp = NF^2;   kd = 2*DR*sqrt(kp);
    Kp = kp*eye(6);  Kd = kd*eye(6);

    % feedback/feedforward/total torque
    ufb = M*(Kp*(q_d - q) + Kd*(dq_d - dq));
    uff = Md*ddq_d + G;
    u = ufb + uff;

    % next state
    ddq = [dq; M\(u - C - G)];
end