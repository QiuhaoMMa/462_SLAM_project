clc;
clear;
close all;

videoFile = 'C:/Users/97895/Desktop/462/Individual final project/video2.mp4';
outputFolder = 'C:/Users/97895/Desktop/462/Individual final project/video_frames';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

vidObj = VideoReader(videoFile);
frameIdx = 1;

while hasFrame(vidObj)
    frame = readFrame(vidObj);
    
    frameRGB = imresize(frame, [1080 1920]);  
    
    fileName = sprintf('frame_%04d.png', frameIdx);
    imwrite(frameRGB, fullfile(outputFolder, fileName));
    frameIdx = frameIdx + 1;
end

disp('finish');

