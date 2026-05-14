

A C++ implementation for processing KITTI raw data, focused on LiDAR-Inertial Odometry (LIO) fundamentals. This project serves as a modular foundation for building a SLAM (Simultaneous Localization and Mapping) pipeline.

![Demo Video](./image/2026-05-02-github.gif)


## 🚀 Current Implementation
The core engine currently handles data ingestion and synchronization for synchronized KITTI sequences:
- **`KittiReader`**: Efficiently parses Velodyne binary point clouds (`.bin`) and handles coordinate transformations.
- **`ImuReader`**: Synchronizes OXTS inertial data (accelerometer/gyroscope) and maps high-precision timestamps.
- **`Calibration`**: Implements the rigid body transformation ($T_{imu \to velo}$) to align inertial frames with the LiDAR center.
- **`Preprocessor`**: Voxel downsampling using hash keys and unordered map (hash map) computing each voxel's centroid to determine the boundaries of the voxel, the deskewing process is also within this file, this is when the car turns or moves then we need to make sure the lidar is still coordinated.
- **`Iterative Closest Point`**: Point to Plane algorithm for aligning point cloud per time stamp by its 10 nearest neighbor's normal (plane) to the previous point cloud time stamp, for understanding where positioning is between each scan for the IMU propagation 
- **`Error-State Kalman Filter`**: Assume biases and noise aware from the IMU (high frequency) tracks the system's true state to propagate the nominal state, while the lidar (lower frequency) measurement update steps in to estimate the actual kinematic errors and sensor biases/noise using $15 \times 15$ state covariance matrices and similarly dimensioned Jacobians derived from LiDAR point-cloud registration. 

## 🛠 Project Roadmap
The development of this repository is focused on evolving from a data reader into a full-scale LIO sensor fusion system. Here are the next steps:

### 1. Visualization & Python Integration
- **Open3D LIO Visualization**: A dedicated Python bridge will stream the processed LIO trajectory.
- **Correcting Orientation of Scan Matching**: Issues with the ICP algorithm and which way to face.

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
Ensure you have **Eigen3** installed. ***Visualization is not working yet**

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
./kitti_lio ../data/2011_09_26/2011_09_26_drive_0070_sync
```

```bash
conda create env
conda activate env
conda install python==3.12

pip install open3d
pip install natsort

python viz/render_video.py
```
