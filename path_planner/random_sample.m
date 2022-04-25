function [xRand, yRand] = random_sample(xMin,yMin,xMax,yMax,bias)
    if nargin < 5
        bias = 1;
    end
    
    xRand = xMin + (xMax-xMin)*rand^bias;
    yRand = yMin + (yMax-yMin)*rand^bias;
end