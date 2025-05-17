##################################################################################################
# Author: Soumyadip Banerjee                                                                     #
# Website: https://www.philosopherscode.de                                                       #
# github:  https://github.com/Nova200019                                                         #
# Disclaimer: This code is for educational purposes and provided "as-is" without any warranties. #
##################################################################################################

% Paths to your files
imagePath = 'C:\\Users\\soumy\\Downloads\\Gmail'; % Path to your images directory
asciiFilePath = 'C:\\Users\\soumy\\Downloads\\check3.txt'; % Path to your ASCII point cloud file
cameraParamsPath = 'C:\\Users\\soumy\\Downloads\\Gmail\\calibration.mat';

% Load camera intrinsic parameters.
intrinsic = load(cameraParamsPath);

% Load images using imageDatastore.
imds = imageDatastore(imagePath);
imageFileNames = imds.Files;

% Load the ASCII point cloud file
data = load(asciiFilePath);

% Check if the data is in the correct format (M-by-3)
[rows, cols] = size(data);
if cols ~= 3
    % Try to clean the data by keeping only the first three columns
    if cols > 3
        data = data(:, 1:3);
    else
        error('Invalid point cloud data format. It should be M-by-3.');
    end
end

% Convert to pointCloud object
ptCloud = pointCloud(data);

% Square size of the checkerboard in mm.
squareSize = 30;

% Set random seed to generate reproducible results.
rng('default')

% Extract checkerboard corners from the images.
[imageCorners3d, checkerboardDimension, dataUsed] = ...
    estimateCheckerboardCorners3d(imageFileNames, intrinsic.cameraParams, squareSize);

% Remove the unused image files.
imageFileNames = imageFileNames(dataUsed);

% Filter the point cloud files that are not used for detection.
ptCloudFileNames = {asciiFilePath}; % As we have only one point cloud file
ptCloudFileNames = ptCloudFileNames(dataUsed);

% Extract ROI from the detected checkerboard image corners.
roi = helperComputeROI(imageCorners3d, 5);

% Extract checkerboard plane from point cloud data.
[lidarCheckerboardPlanes, framesUsed, indices] = detectRectangularPlanePoints( ...
    ptCloudFileNames, checkerboardDimension, 'RemoveGround', true, 'ROI', roi);

% Filter the data based on frames used.
imageCorners3d = imageCorners3d(:, :, framesUsed);
ptCloudFileNames = ptCloudFileNames(framesUsed);
imageFileNames = imageFileNames(framesUsed);

% Ensure lidarCheckerboardPlanes is of type pointCloud
if ~iscell(lidarCheckerboardPlanes)
    lidarCheckerboardPlanes = {lidarCheckerboardPlanes};
end

% Convert any double type point clouds to pointCloud objects
for i = 1:length(lidarCheckerboardPlanes)
    if isa(lidarCheckerboardPlanes{i}, 'double')
        % Ensure the data is in M-by-3 format
        [rows, cols] = size(lidarCheckerboardPlanes{i});
        if cols == 3
            lidarCheckerboardPlanes{i} = pointCloud(lidarCheckerboardPlanes{i});
        elseif rows == 3
            lidarCheckerboardPlanes{i} = pointCloud(lidarCheckerboardPlanes{i}');
        else
            error('Invalid point cloud data format. It should be M-by-3 or M-by-N-by-3.');
        end
    end
end

% Estimate the LiDAR-camera transform.
[tform, errors] = estimateLidarCameraTransform(lidarCheckerboardPlanes, ...
    imageCorners3d, intrinsic.cameraParams);

% Fuse LiDAR and camera data.
helperFuseLidarCamera(imageFileNames, ptCloudFileNames, indices, ...
    intrinsic.cameraParams, tform);
