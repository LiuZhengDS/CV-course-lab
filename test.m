% Open the specified folder and read images. 
    directory = './TeddyBearPNG/'; % Path to your local image directory 
    Files=dir(strcat(directory, '*.png'));
    n = length(Files);

    % Apply normalized 8-point RANSAC algorithm to find best matches. (Lab assignment 3+5)
    % The output includes cell arrays with Coordinates (C), Descriptors (D) and indies (Matches)for all matched pairs.
    disp('ransac_match');
    if exist(strcat(directory, 'Matches.mat')) && exist(strcat(directory, 'C.mat'))
        load(strcat(directory, 'Matches.mat'));
        load(strcat(directory, 'C.mat'));
    else
        [C,D, Matches] = ransac_match(directory) 
        save(strcat(directory, 'Matches.mat'), 'Matches');
        save(strcat(directory, 'C.mat'), 'C');
    end

    % Chaining: Create point-view matrix (PV) to represent point correspondences 
    % for different camera views (Lab assignment 6).
    disp('chainimages');
    if exist(strcat(directory, 'PV.mat'))
        load(strcat(directory, 'PV.mat'));
    else
        [PV] = chainimages(Matches);
        save(strcat(directory, 'PV.mat'), 'PV');
    end


    % Stitching: with affine Structure from Motion
    % Stitch every 3 images together to create a point cloud.
    Clouds = {};
    i = 1;
    numFrames = 3;

    for iBegin = 1:n-(numFrames - 1)
        iEnd = iBegin + numFrames - 1;
        
        % Select frames from the PV matrix to form a block
        block = PV(iBegin:iEnd,:);
        
        % Select columns from the block that do not have any zeros
        colInds = find((block(1,:).*block(2,:).*block(3,:))~=0);
        
        % Check the number of visible points in all views
        numPoints = size(colInds, 2);
        if numPoints < 8
            continue
        end
        
        % Create measurement matrix X with coordinates instead of indices using the block and the 
        % Coordinates C 
        block = block(:, colInds);
        X = zeros(2 * numFrames, numPoints);
        for f = 1:numFrames
            for p = 1:numPoints
                c = C{iBegin+f-1};
                X(2 * f - 1, p) = c(1,block(f,p)); % x
                X(2 * f, p)     = c(2,block(f,p)); % y
            end
        end
        
        % Estimate 3D coordinates of each block following Lab 4 "Structure from Motion" to compute the M and S matrix.
        % Here, an additional output "p" is added to deal with the non-positive matrix error
        % Please check the chol function inside sfm.m for detail.
        [M, S, p] = sfm(X);
            % Your structure from motion implementation for the measurements X

        if ~p
            % Compose Clouds in the form of (M,S,colInds)
            Clouds(i, :) = {M, S, colInds};
            i = i + 1;
        end
    end
    
    
    
    %%
    
    
% Create some random points in two dimensions
       n = 10;
       X = normrnd(0, 1, [n 2]);
 
       % Those same points, rotated, scaled, translated, plus some noise
       S = [0.5 -sqrt(3)/2; sqrt(3)/2 0.5]; % rotate 60 degrees
       Y = normrnd(0.5*X*S + 2, 0.05, n, 2);
 
       % Conform Y to X, plot original X and Y, and transformed Y
       [d, Z, tr] = procrustes(X,Y);
       plot(X(:,1),X(:,2),'rx', Y(:,1),Y(:,2),'b.', Z(:,1),Z(:,2),'bx');    