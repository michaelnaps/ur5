function  c_test = collision_detector(xNear,yNear,xNew, yNew,obstacles)
    % we test 10 uniformally distributed points along the line that
    % connects (xNear,yNear) to (xNew,yNew)
    s = 0:0.05:1;
    xq = xNear + s.*(xNew-xNear);
    yq = yNear + s.*(yNew-yNear);
    
    c_test = 0; % initialize as false (no collision)
    for i=1:length(obstacles)
        % check if any point is within the polygon
        in = inpolygon(xq,yq,obstacles(i).xv,obstacles(i).yv);
        if any(in) % if any one of the points are within the polygon
            c_test = 1; % set it true (collision)
            break;      % break the loop, no need to test other areas
        end
    end
    
end