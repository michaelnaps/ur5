function [M, T, S] = MTS_ur5(joints)
    %% Compute the orientation and position of EE-origin
    % only works if ALL joints revolute
    % construct home position matrix
    T = NaN(4, 4, length(joints));
    BR = [0,0,0,1];
    
    j = 1;
    S = NaN(6,length(joints)-2);
    
    T(:,:,1) = eye(4);
    for i = 2:length(joints)
        % home position matrix
        T(:,:,i) = T(:,:,i-1)*[joints(i).R, joints(i).Offset'; BR];
    
        % skew matrices
        if (~isempty(joints(i).Axis))
            AdT = Adjoint(T(:,:,i));
            S(:,j) = AdT*[joints(i).Axis'; zeros(3,1)];
            j = j + 1;
        end
    end
    
    M = T(:,:,end);
end

