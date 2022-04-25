function [R] = rotMat(axis, theta)
    switch axis
        case 'x'
            R = [
                1, 0, 0;
                0, cos(theta), -sin(theta);
                0, sin(theta),  cos(theta);
            ];
            return;
        case 'y'
            R = [
                 cos(theta), 0, sin(theta);
                 0, 1, 0;
                -sin(theta), 0, cos(theta);
            ];
            return;
        case 'z'
            R = [
                cos(theta), -sin(theta), 0;
                sin(theta),  cos(theta), 0;
                0, 0, 1;
            ];
            return;
        otherwise
            fprintf("Invalid axis of rotation.\n")
            R = -1;
            return;
    end

end