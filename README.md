# Tumor-Motion-Tracking
# Tumor Motion Tracking (Simulation + Kalman Filter)

## 1. Overview
This project simulates tumor motion in synthetic 2D medical imaging data and tracks the tumor across frames using **Kalman filtering**.  
The goal is to demonstrate how classical tracking algorithms can be applied in a medical context, where accurate tumor localization is critical for treatments such as **radiotherapy** and **motion-compensated imaging**.  

We implemented and compared different tracking strategies, starting with optical flow (Lucas-Kanade), then moving to centroid-based detection combined with **Kalman filtering** for robust performance.

---

## 2. Methods

### 1. **Synthetic Tumor Motion Simulation**
- A sinusoidal-like tumor trajectory is generated (`tumor_simulation.m`).  
  **Reason:** I modeled tumor motion as a sinusoidal trajectory along the x-axis to mimic respiratory-induced displacement. Breathing motion is approximately periodic and can be well-approximated by a sinusoidal function, making it both clinically relevant and mathematically convenient for synthetic tumor simulation.

- Ground truth trajectory is saved in `trajectory_truth.csv`.  
- A sequence of frames (`frames.mat`) is generated to mimic real imaging.

### 2. **Tumor Tracking Approaches**
#### (a) Lucas-Kanade Optical Flow (Explored, not suitable)
- Implemented with `LucasKanadeOpticalFlow.m` and `LucasKanadeStep.m`.  
- Failed in this context due to:
  - **Small object problem** → tumor is small compared to the image, so Lucas-Kanade’s local intensity window is unreliable.  
  - **Low contrast** → synthetic tumor blob has uniform intensity; optical flow assumes rich texture/gradients.  
  - **Biological mismatch** → tumor motion is smooth and global (respiration-driven), but Lucas-Kanade is designed for tracking *local texture displacements*.  
- *Kept in repo for completeness, but not used in final results.*

#### (b) Centroid Detection
- The tumor is segmented via simple intensity thresholding.
- The centroid of the largest detected region is extracted as the tumor’s position.
- Works, but noisy without filtering.

#### (c) Kalman Filter (Final Method)
- Two models were tested:
  - **Constant-Velocity Kalman Filter**
  - **Constant-Acceleration Kalman Filter** (improved)  
- The constant-acceleration model gave the best alignment with ground truth.  

---

## 3. Results

- **Trajectory plots:**  
  - `Kalman_plot.fig` and `Klman_plot_1.fig` compare ground truth vs. Kalman filter prediction.  
  - The constant-acceleration Kalman filter showed smooth, realistic tracking.

- **Tracked data:**  
  - `trajectory_tracked_kalman_accel.csv` stores the predicted trajectory.  

- **Simulation Video:**  
  - `tumor_tracking_kalman_accel.avi`  
  - Shows frame-by-frame tumor tracking with a red marker.

---

## 4. Key Insights

### Why Lucas-Kanade Did Not Work 
- Tumor motion in lungs is a **global, periodic oscillation** (caused by respiration).  
- Lucas-Kanade assumes **local intensity variations** (edges, texture), which tumors often lack in medical imaging.  
- As a result, LK produced unstable or meaningless flow fields in this setting.  

### Why Constant-Acceleration Kalman Filter Works Better 
- Real tumor motion is not uniform velocity.  
  - During respiration, the tumor **slows near turning points** (end-inhale/exhale) and **accelerates in between**.  
  - This acceleration–deceleration pattern is central to breathing motion.  
- A **constant-velocity model** fails to capture this change in speed, leading to drift.  
- A **constant-acceleration model** mimics this **biologically realistic respiratory motion**, producing more accurate tracking.  

---


## 5. File Descriptions
- **simulate_tumor.m** – Generates synthetic sinusoidal tumor motion and saves:
  - `frames.mat` (synthetic frames)
  - `trajectory_truth.csv` (ground-truth trajectory)

- **track_tumor.m** – Runs Lucas–Kanade optical flow tumor tracking (baseline, exploratory).  
  → Produces `trajectory_tracked.csv`.

- **kalman_tumor_track.m** – First Kalman filter tracker implementation (constant velocity assumption).  

- **track_tumor_kalman_1.m** – Improved Kalman filter tracker (constant acceleration, better biological fit).  
  → Produces `trajectory_tracked_kalman_accel.csv` and `tumor_tracking_kalman_accel.avi`.

- **LucasKanadeOpticalFlow.m** – Implementation of Lucas–Kanade optical flow algorithm.  
- **LucasKanadeStep.m** – Helper step for Lucas–Kanade updates.  
- **WarpImage.m** – Utility for image warping in the optical flow pipeline.  

- **trajectory_truth.csv** – Ground-truth tumor trajectory (from simulation).  
- **trajectory_tracked.csv** – Lucas–Kanade estimated trajectory.  
- **trajectory_tracked_kalman_accel.csv** – Kalman filter (constant acceleration) estimated trajectory.  

- **frames.mat** – MATLAB data file containing generated synthetic frames.  
- **Kalman_plot.fig** – MATLAB figure of Kalman vs truth (velocity model).  
- **Klman_plot_1.fig** – MATLAB figure of improved Kalman vs truth (acceleration model).  
- **LK.fig** – MATLAB figure of Lucas–Kanade tracking attempt.  
- **tumor_tracking_kalman_accel.avi** – Video demo of Kalman tracking overlay.  

## 6. Future Work

- Incorporating **Physics-Informed Neural Networks (PINNs)** to model tumor dynamics more accurately.  
- Extending from **2D synthetic motion** to **3D volumetric tumor tracking** (closer to clinical imaging).  
- Testing with **real patient respiratory motion data** to validate robustness.  

















