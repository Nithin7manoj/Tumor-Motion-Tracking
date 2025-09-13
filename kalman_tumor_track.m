% track_tumor_kalman_accel.m
% Kalman (constant-acceleration) + centroid tracking for synthetic tumor
% Requires: frames.mat, trajectory_truth.csv

%% Load synthetic data
load('frames.mat');                         % frames (H x W x N)
truth = readmatrix('trajectory_truth.csv'); % ground truth [t,x,y]
numFrames = size(frames,3);

%% Kalman filter (constant-acceleration) setup
dt = 1; % time step = 1 frame

% State: [x; y; vx; vy; ax; ay]
F = [1 0 dt 0 0.5*dt^2 0;
     0 1 0 dt 0 0.5*dt^2;
     0 0 1 0 dt 0;
     0 0 0 1 0 dt;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

% measurement: we only observe position (x,y)
H = [1 0 0 0 0 0;
     0 1 0 0 0 0];

% Process & measurement noise (tuneable)
Q = 5e-3 * eye(6);    % process noise covariance (6x6)
R = 0.5 * eye(2);     % measurement noise covariance (2x2)

% Initial state (use truth position, zero vel/acc)
x = [truth(1,2); truth(1,3); 0; 0; 0; 0]; % 6x1
P = eye(6);                                % 6x6 covariance

%% Preallocate storage
tracked = zeros(numFrames,3);
tracked(1,:) = [1, x(1), x(2)];

%% Video writer (optional)
makeVideo = true;
if makeVideo
    v = VideoWriter('tumor_tracking_kalman_accel.avi');
    open(v);
end

%% Main loop
for t = 2:numFrames
    frame = frames(:,:,t);

    % --- Measurement via centroid detection ---
    bw = frame > 0.5; % binary mask (synthetic data)
    stats = regionprops(bw,'Area','Centroid');
    if ~isempty(stats)
        [~, idx] = max([stats.Area]);
        meas = stats(idx).Centroid; % [x,y]
    else
        meas = [NaN, NaN]; % missing measurement
    end

    % --- Prediction ---
    x_pred = F * x;                 % 6x1
    P_pred = F * P * F' + Q;        % 6x6

    % --- Update (if measurement available) ---
    if ~isnan(meas(1))
        z = meas(:); % 2x1
        S = H * P_pred * H' + R;               % 2x2
        K = P_pred * H' / S;                   % 6x2
        x = x_pred + K * (z - H * x_pred);     % 6x1
        P = (eye(6) - K * H) * P_pred;         % 6x6
    else
        x = x_pred;
        P = P_pred;
    end

    % Save result
    tracked(t,:) = [t, x(1), x(2)];

    % --- Video overlay ---
    if makeVideo
        frmRGB = repmat(uint8(frame*255), [1 1 3]);
        frmRGB = insertMarker(frmRGB, [x(1), x(2)], 'x', 'Color', 'red', 'Size', 10);
        writeVideo(v, frmRGB);
    end

    % Progress
    if mod(t,10) == 0
        disp(['Processed frame ' num2str(t) ' of ' num2str(numFrames)]);
    end
end

if makeVideo
    close(v);
    disp('Tracking video saved: tumor_tracking_kalman_accel.avi');
end

%% Save & plot
writematrix(tracked,'trajectory_tracked_kalman_accel.csv');
disp('Kalman (accel) tracking complete: trajectory_tracked_kalman_accel.csv saved.');

figure;
plot(truth(:,1), truth(:,2),'g-','LineWidth',2); hold on;
plot(tracked(:,1), tracked(:,2),'r--','LineWidth',1.5);
xlabel('Frame'); ylabel('X Position (px)');
legend('Ground Truth','Kalman-Accel + Centroid');
title('Tumor Tracking (Kalman constant-acceleration)');
grid on;
