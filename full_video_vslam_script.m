
clc; clear; close all;

%% Load Extracted Frames
imageFolder = "video_frames"; % Update this to the full path if needed
imds = imageDatastore(imageFolder, 'FileExtensions', {'.jpg', '.jpeg', '.png'});
currFrameIdx = 1;
currI = readimage(imds, currFrameIdx);

%% Camera Intrinsics (placeholder, replace with calibrated values)
focalLength = [535.4, 539.2];
principalPoint = [320.1, 247.6];
imageSize = size(currI,[1 2]);
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

%% Feature Extraction Parameters
scaleFactor = 1.2;
numLevels = 8;
numPoints = 1000;
rng(0)

%% Initial Feature Extraction
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints);
firstI = currI;
currFrameIdx = currFrameIdx + 1;
isMapInitialized = false;

while ~isMapInitialized && currFrameIdx < numel(imds.Files)
    currI = readimage(imds, currFrameIdx);
    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints);
    currFrameIdx = currFrameIdx + 1;
    indexPairs = matchFeatures(preFeatures, currFeatures, 'Unique', true, 'MaxRatio', 0.9, 'MatchThreshold', 40);
    if size(indexPairs, 1) < 100, continue; end
    preMatchedPoints = prePoints(indexPairs(:,1), :);
    currMatchedPoints = currPoints(indexPairs(:,2), :);
    [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);
    [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints, intrinsics);
    ratio = scoreH / (scoreH + scoreF);
    if ratio > 0.45
        inlierTformIdx = inliersIdxH;
        tform = tformH;
    else
        inlierTformIdx = inliersIdxF;
        tform = tformF;
    end
    inlierPrePoints = preMatchedPoints(inlierTformIdx);
    inlierCurrPoints = currMatchedPoints(inlierTformIdx);
    [relPose, validFraction] = estrelpose(tform, intrinsics, inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));
    if validFraction < 0.9 || numel(relPose) > 1, continue; end
    [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(rigidtform3d, relPose, ...
        inlierPrePoints, inlierCurrPoints, intrinsics, 1);
    if ~isValid, continue; end
    indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx), :);
    isMapInitialized = true;
    disp(['Map initialized using frame 1 and frame ', num2str(currFrameIdx - 1)])
end

if ~isMapInitialized
    error("Map initialization failed.");
end

% Visualization
showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), "Montage");
title("Map Initialization Matches");

%% Initialize View Sets
vSetKeyFrames = imageviewset;
mapPointSet = worldpointset;
preViewId = 1;
currViewId = 2;

vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigidtform3d, Points=prePoints, Features=preFeatures.Features);
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, Points=currPoints, Features=currFeatures.Features);
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, Matches=indexPairs);
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);
mapPointSet = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));
mapPointSet = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));

%% Bundle Adjustment
tracks = findTracks(vSetKeyFrames);
cameraPoses = poses(vSetKeyFrames);
[refinedPoints, refinedAbsPoses] = bundleAdjustment(xyzWorldPoints, tracks, cameraPoses, intrinsics, ...
    FixedViewIDs=1, PointsUndistorted=true, AbsoluteTolerance=1e-7, RelativeTolerance=1e-15, ...
    MaxIteration=20, Solver="preconditioned-conjugate-gradient");

medianDepth = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;
refinedAbsPoses.AbsolutePose(currViewId).Translation = refinedAbsPoses.AbsolutePose(currViewId).Translation / medianDepth;
relPose.Translation = relPose.Translation / medianDepth;

vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, relPose);
mapPointSet = updateWorldPoints(mapPointSet, newPointIdx, refinedPoints);
mapPointSet = updateLimitsAndDirection(mapPointSet, newPointIdx, vSetKeyFrames.Views);
mapPointSet = updateRepresentativeView(mapPointSet, newPointIdx, vSetKeyFrames.Views);

% Visualization
featurePlot = helperVisualizeMatchedFeatures(currI, currPoints(indexPairs(:,2)));
mapPlot = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPointSet);
showLegend(mapPlot);
