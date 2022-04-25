function [dq] = fdm(q, dt)
    N  = length(q);
    dq = zeros(size(q));
    for i = 1:N
        % three point forward difference
        if i < 3
            dq(:,i) = (-3*q(:,i) + 4*q(:,i+1) - q(:,i+2))/(2*dt);
        % three point backward difference
        elseif i > (N-3)
            dq(:,i) =  (q(:,i-2) - 4*q(:,i-1) + 3*q(:,i))/(2*dt);
        % 4 point central difference
        else
            dq(:,i) = (q(:,i-2) - 8*q(:,i-1) + 8*q(:,i+1) - q(:,i+2))/(12*dt);
        end
    end
    
end