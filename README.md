

A C++ implementation for processing KITTI raw data, focused on LiDAR-Inertial Odometry (LIO) fundamentals. This project serves as a modular foundation for building a SLAM (Simultaneous Localization and Mapping) pipeline.

## 🚀 Current Implementation
The core engine currently handles data ingestion and synchronization for synchronized KITTI sequences:
- **`KittiReader`**: Efficiently parses Velodyne binary point clouds (`.bin`) and handles coordinate transformations.
- **`ImuReader`**: Synchronizes OXTS inertial data (accelerometer/gyroscope) and maps high-precision timestamps.
- **`Calibration`**: Implements the rigid body transformation ($T_{imu \to velo}$) to align inertial frames with the LiDAR center.

## 🛠 Project Roadmap
The development of this repository is focused on evolving from a data reader into a full-scale LIO sensor fusion system. Here are the next steps:

### 1. Spatial Indexing & Pre-processing
- **Voxel Hash Downsampling**: To maintain real-time performance, point clouds are "summarized" into a 3D grid (voxels). Instead of processing every point, we average points within a voxel. Using a **Hash Map** for these voxels allows $O(1)$ lookup times, significantly outperforming dense grids.
- **LiDAR Deskewing**: Since the LiDAR sensor moves while it spins, the first point in a scan and the last point are captured from different spatial positions. By using IMU data to interpolate the exact motion during the scan (the "sweep"), we "un-distort" the cloud to a single time-point.

### 2. Scan Matching & Optimization
- **ICP (Iterative Closest Point)**: This is the geometric alignment phase. The algorithm minimizes the distance between two point clouds by iteratively rotating and translating the source cloud until it "snaps" into the target cloud.
- **kd-tree (via `nanoflann`)**: To make ICP fast, we need to find the "nearest neighbor" for thousands of points. A kd-tree (k-dimensional tree) partitions space into regions, allowing us to find the closest point in $O(\log n)$ time rather than checking every single point.

### 3. State Estimation (The Kalman Filter)
The **Extended Kalman Filter (EKF)** acts as the brain of the fusion:
- **Prediction Phase**: Uses high-frequency IMU data to guess where the robot moved (Kinematics).
- **Update Phase**: Uses the ICP result (LiDAR) to correct the IMU's drift. 
- **Math Intuition**: Think of it as a weighted average where weights are based on "uncertainty." If the IMU is noisy, we trust the LiDAR more; if the LiDAR is in a featureless hallway, we trust the IMU's motion model more.

### 4. Visualization & Python Integration
- **Open3D LIO Visualization**: A dedicated Python bridge will stream the processed LIO trajectory.
- **Lidar Stream Overlay**: We plan to create a dashboard showing the estimated 3D trajectory (LIO) in real-time, providing a "God's eye view" of the sensor's path.

---

## 🧠 Technical

### Coordinate Transformations
In this project, we deal with "Rigid Body Transformations." A point $P$ in the IMU frame is mapped to the LiDAR frame $V$ using:
$$P_V = R_{imu \to velo} \cdot P_{imu} + t_{imu \to velo}$$
Where $R$ is a $3 \times 3$ rotation matrix and $t$ is a translation vector. In the C++ implementation, we combine these into a single $4 \times 4$ **Eigen** matrix for computational efficiency.

### C++ Implementation Ideas
- **Memory Safety**: Utilizing `std::filesystem` namespace and smart pointers to ensure robust file I/O and memory management.
- **Binary Efficiency**: Point clouds are read as raw binary buffers. Instead of parsing strings, we map the binary file directly into memory structures, which is orders of magnitude faster for LiDAR data which can contain 100k+ points per frame.
- **Modular Design**: The readers are decoupled from the estimators. This allows you to swap the KITTI reader for a ROS-bag reader or a live sensor driver without changing the core SLAM logic, further work will be done.

---

## 💻 Building the Project
Ensure you have **Eigen3** installed.

```bash
mkdir build && cd build
cmake ..
make
./test_kitti
