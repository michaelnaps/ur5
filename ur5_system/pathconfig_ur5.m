function [qList] = pathconfig_ur5(home, path)
    %% path cariables
    xList = path.x;
    yList = path.y;
    zList = path.z;
    RList = path.R;

    %% home config setup
    M = home.M;
    Slist = home.Slist;

    %% variable setup
    Nx = length(xList);
    BR = [0,0,0,1];
    qList = ones(Nx,6);

    for i = 1:Nx
        Tgoal = [
            RList(:,:,i), [xList(i); yList(i); zList(i)];
            BR;
        ];
    
        s = 0;
        s_count = 0;
        while ~s
            if i==1 || s_count > 0
                qRand = randn(6,1);
                [qList(i,:), s] = inverseKinematics_ur5(qRand, Slist, M, Tgoal, 1e-6, 1e-6);
            else
                [qList(i,:), s] = inverseKinematics_ur5(qList(i-1,:)', Slist, M, Tgoal, 1e-6, 1e-6);
            end
    
            s_count = s_count + 1;
        end
    end
end

