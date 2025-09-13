% simulate_tumor.m
% Simulates synthetic tumor motion (sinusoidal path) and saves frames + trajectory
% Author: [Your Name], Sept 2025

%% Parameters
numFrames = 100;     % number of frames
imgSize = 256;       % image size (256x256 pixels)
radius = 12;         % tumor radius in pixels
amplitude = 60;      % max displacement along x-axis
freq = 0.1;          % sinusoidal frequency
y_center = 128;      % fixed y center (tumor stays horizontal)

%% Preallocate storage
frames = zeros(imgSize, imgSize, numFrames); % video frames
truthTrajectory = zeros(numFrames, 3);       % [time, x, y]

%% Generate frames
for t = 1:numFrames
    % Tumor center moves sinusoidally along x
    x_center = 128 + amplitude * sin(2*pi*freq*t);
    
    % Create blank frame
    frame = zeros(imgSize, imgSize);
    
    % Create tumor mask (circle)
    [X,Y] = meshgrid(1:imgSize, 1:imgSize);
    mask = (X - x_center).^2 + (Y - y_center).^2 <= radius^2;
    
    % Place tumor in frame
    frame(mask) = 1;
    
    % Save frame + trajectory
    frames(:,:,t) = frame;
    truthTrajectory(t,:) = [t, x_center, y_center];
end

%% Save outputs
writematrix(truthTrajectory, 'trajectory_truth.csv'); % save trajectory
save('frames.mat','frames');                          % save frames

%% Quick visualization
figure;
imshow(frames(:,:,1));
title('Sample Tumor Frame (t=1)');

disp('Simulation complete: frames.mat + trajectory_truth.csv saved.');
