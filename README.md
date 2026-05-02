

A C++ implementation for processing KITTI raw data, focused on LiDAR-Inertial Odometry (LIO) fundamentals. This project serves as a modular foundation for building a SLAM (Simultaneous Localization and Mapping) pipeline.

![Demo Video](./image/2026-05-02-github.gif)


## 🚀 Current Implementation
The core engine currently handles data ingestion and synchronization for synchronized KITTI sequences:
- **`KittiReader`**: Efficiently parses Velodyne binary point clouds (`.bin`) and handles coordinate transformations.
- **`ImuReader`**: Synchronizes OXTS inertial data (accelerometer/gyroscope) and maps high-precision timestamps.
- **`Calibration`**: Implements the rigid body transformation ($T_{imu \to velo}$) to align inertial frames with the LiDAR center.
- **`Preprocessor`**: Voxel downsampling using hash keys and unordered map (hash map) computing each voxel's centroid to determine the boundaries of the voxel, the deskewing process is also within this file, this is when the car turns or moves then we need to make sure the lidar is still coordinated.

## 🛠 Project Roadmap
The development of this repository is focused on evolving from a data reader into a full-scale LIO sensor fusion system. Here are the next steps:

### 1. Scan Matching & Optimization
- **ICP (Iterative Closest Point)**: This is the geometric alignment phase. The algorithm minimizes the distance between two point clouds by iteratively rotating and translating the source cloud until it "snaps" into the target cloud.
- **kd-tree (via `nanoflann`)**: To make ICP fast, we need to find the "nearest neighbor" for thousands of points. A kd-tree (k-dimensional tree) partitions space into regions, allowing us to find the closest point in $O(\log n)$ time rather than checking every single point.

### 2. State Estimation (The Kalman Filter)
The **Extended Kalman Filter (EKF)** acts as the brain of the fusion:
- **Prediction Phase**: Uses high-frequency IMU data to guess where the robot moved (Kinematics).
- **Update Phase**: Uses the ICP result (LiDAR) to correct the IMU's drift. 
- **Math Intuition**: Think of it as a weighted average where weights are based on "uncertainty." If the IMU is noisy, we trust the LiDAR more; if the LiDAR is in a featureless hallway, we trust the IMU's motion model more.

### 3. Visualization & Python Integration
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
Ensure you have **Eigen3** installed. **No cmakelists.txt file yet!**

```bash
mkdir build && cd build
cmake ..
make
./test_kitti
```

```bash
conda create env
conda activate env
conda install python==3.12

pip install open3d
pip install natsort

python viz/render_video.py
```
