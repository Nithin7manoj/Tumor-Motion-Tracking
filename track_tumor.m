% track_tumor.m
% Tracks tumor motion using Lucas-Kanade Optical Flow
% Requires: frames.mat, trajectory_truth.csv, LucasKanadeOpticalFlow.m

%% Load data
load('frames.mat'); % synthetic frames
truthTrajectory = readmatrix('trajectory_truth.csv');
numFrames = size(frames,3);

% Initial tumor position (from ground truth)
x0 = truthTrajectory(1,2);
y0 = truthTrajectory(1,3);

% Store tracked positions
trackedTrajectory = zeros(numFrames,3);
trackedTrajectory(1,:) = [1, x0, y0];

%% Parameters for Lucas-Kanade
WindowSize = 15;
MaxIter = 3;
NumLevels = 3;

%% Loop through frames
for t = 2:numFrames
    I1 = frames(:,:,t-1);
    I2 = frames(:,:,t);
    
    % Optical flow field
    [u,v] = LucasKanadeOpticalFlow(I1,I2,WindowSize,MaxIter,NumLevels);
    
    % Define mask around last known tumor position
    [X,Y] = meshgrid(1:size(I1,2),1:size(I1,1));
    mask = (X-x0).^2 + (Y-y0).^2 <= 15^2; % radius=15 pixels
    
    % Average flow in tumor region
    u_mean = mean(u(mask));
    v_mean = mean(v(mask));
    
    % Update tumor position
    x0 = x0 + u_mean;
    y0 = y0 + v_mean;
    
    % Save tracked position
    trackedTrajectory(t,:) = [t, x0, y0];

    if mod(t,10) == 0
        disp(['Processed frame ' num2str(t) ' of ' num2str(numFrames)]);
    end
end

%% Save results
writematrix(trackedTrajectory, 'trajectory_tracked.csv');
disp('Tracking complete: trajectory_tracked.csv saved.');

%% Quick visualization
figure;
plot(truthTrajectory(:,1), truthTrajectory(:,2), 'g-', 'LineWidth', 2); hold on;
plot(trackedTrajectory(:,1), trackedTrajectory(:,2), 'r--', 'LineWidth', 2);
xlabel('Frame'); ylabel('X Position (px)');
legend('Ground Truth','Tracked');
title('Tumor Motion Tracking');
grid on;
