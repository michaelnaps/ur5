function [home, joints, links] = home_ur5(filename)
    %% Load Data
    [links, joints] = load_urdf(filename);
    nJoints = length(joints);
    nLinks = length(links);
    nAct = length(links)-1;
    
    %% M, T, and S data
    [M, Tlist, Slist] = MTS_ur5(joints);

    %% Home Position and Gravity Matrices
    BR = [0, 0, 0, 1];
    Mlist = zeros(4,4,nAct+1);
    Glist = zeros(6,6,nAct);
    
    joints(1).M = eye(4);
    
    for i = 2:nJoints
        M_i = [joints(i).R, joints(i).Offset'; BR];
        joints(i).M = joints(i-1).M*M_i;
    end
    
    links(1).M = eye(4);
    for i = 2:nLinks
        T_i = [links(i).R, links(i).Offset'; BR];
        links(i).M = joints(i).M*T_i;
        Mlist(:,:,i-1) = links(i-1).M \ links(i).M;
    end
    
    for i = 2:nJoints-1
        Glist(:,:,i-1) = [links(i).Inertia, zeros(3); zeros(3), links(i).Mass*eye(3)];
    end

    home.M = M;
    home.Tlist = Tlist;
    home.Slist = Slist;
    home.Mlist = Mlist;
    home.Glist = Glist;
end