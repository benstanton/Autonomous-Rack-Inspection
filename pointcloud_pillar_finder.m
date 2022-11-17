clc
close all
clear all
tic

cloudMatrix = noisyMatrix(100, 100, 100, 1000);
cloudMatrix = addRandomPillar(cloudMatrix, .5, 0.5, 3, 0.5);

pc = pointCloud(cloudMatrix);
pcshow(pc)

[xGuess, yGuess] = findPillar(pc, 10)




toc

function [xGuess, yGuess] = findPillar(pc, searchStep)
    
    %getting boundaries
    bounds = max(pc.Location);
    xMax = bounds(1);
    yMax = bounds(2);
    zMax = bounds(3);
   
    %initializing guesses and best counter
    xGuess = -1;
    yGuess = -1;
    best = 0;


    %itterating through point cloud
    for x = 0:searchStep:xMax-searchStep
        for y = 0:searchStep:yMax-searchStep
            roi = [x, x+searchStep, y, y+searchStep, 0, zMax];
            indices = findPointsInROI(pc, roi);
            search = select(pc, indices);
            found = sum(search.Location()~=0);

            if found > best
                best = found;
                xGuess = x;
                yGuess = y;
            end
        end

    end

     %displaying best find
     roi = [xGuess, xGuess+searchStep, yGuess, yGuess+searchStep, 0, zMax];
     indices = findPointsInROI(pc, roi);
     search = select(pc, indices);
     figure
     pcshow(search.Location, 'r')
    
end

%generates pointcloud with some noise
function cloud = noisyMatrix(xMax, yMax, zMax, numPoints)
    cloud = zeros(numPoints, 3);
    
    for i = 1:numPoints
        cloud(i, 1) = xMax*rand();
        cloud(i, 2) = yMax*rand();
        cloud(i, 3) = zMax*rand();
    end
end

%adds random pillar of specified parameters
function modMatrix = addRandomPillar(originalMatrix, stepHeight, error, width, stepWidth)
    
    %gettings bounds
    bounds = max(originalMatrix);

    %setting furthest corner from origin as base corner
    xPos = bounds(1)*rand();
    yPos = bounds(2)*rand();

    % setting number of vertical points in base corner 
    numPillarPoints = floor(bounds(3)/stepHeight);

    %defining corner matricies
    baseCornerMatrix = zeros(numPillarPoints, 3);
    cornerMatrixNegX = zeros(numPillarPoints, 3);
    cornerMatrixNegY = zeros(numPillarPoints, 3);
    cornerMatrixNegXY = zeros(numPillarPoints, 3);

    % filling corner matricies
    for i = 1:numPillarPoints
        
        %filling base corner
        baseCornerMatrix(i, 1) = xPos;
        baseCornerMatrix(i, 2) = yPos;
        baseCornerMatrix(i, 3) = i*stepHeight;
        if rand() < error
            baseCornerMatrix(i, :) = [];
        end

        %filling neg X corner
        cornerMatrixNegX(i, 1) = xPos-width;
        cornerMatrixNegX(i, 2) = yPos;
        cornerMatrixNegX(i, 3) = i*stepHeight;
        if rand() < error
            cornerMatrixNegX(i, :) = [];
        end

        %filling neg Y corner
        cornerMatrixNegY(i, 1) = xPos;
        cornerMatrixNegY(i, 2) = yPos - width;
        cornerMatrixNegY(i, 3) = i*stepHeight;
        if rand() < error
            cornerMatrixNegY(i, :) = [];
        end        

        %filling neg XY corner
        cornerMatrixNegXY(i, 1) = xPos-width;
        cornerMatrixNegXY(i, 2) = yPos-width;
        cornerMatrixNegXY(i, 3) = i*stepHeight;
        if rand() < error
            cornerMatrixNegXY(i, :) = [];
        end        

    end    
    
    % adding corners to matrix
    modMatrix = [originalMatrix; 
                baseCornerMatrix; 
                cornerMatrixNegX; 
                cornerMatrixNegY; 
                cornerMatrixNegXY];


    % filling gap between corners
    numGapPoints = ceil(width/stepWidth) - 1;
   
    appendMatrix = zeros(numGapPoints*numPillarPoints*4, 3);

    row = 1;
    for i = 1:numGapPoints % each gap point
        for j = 1:4 % each cardinal direction
            for k = 1:numPillarPoints % each pillar point
                switch j
                    case 1 %neg X
                        appendMatrix(row, 1) = xPos - i*stepWidth;
                        appendMatrix(row, 2) = yPos;
                        appendMatrix(row, 3) = k*stepHeight;

                    case 2 %neg Y
                        appendMatrix(row, 1) = xPos;
                        appendMatrix(row, 2) = yPos - i*stepWidth;
                        appendMatrix(row, 3) = k*stepHeight;

                    case 3 %neg XY
                        appendMatrix(row, 1) = xPos - width;
                        appendMatrix(row, 2) = yPos - i*stepWidth;
                        appendMatrix(row, 3) = k*stepHeight;

                    case 4 %neg YX
                        appendMatrix(row, 1) = xPos - i*stepWidth;
                        appendMatrix(row, 2) = yPos - width;
                        appendMatrix(row, 3) = k*stepHeight;
     
                end

                if rand() < error
                    appendMatrix(row, :) = [];
                end

                row = row + 1;
            end         
        end
    end

    modMatrix = [modMatrix; appendMatrix];
end