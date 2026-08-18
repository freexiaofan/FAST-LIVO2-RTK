# FAST-LIVO2-RTK

FAST-LIVO2-RTK extends FAST-LIVO2 with RTK/GNSS-constrained global optimization for long-term LiDAR-visual mapping.

**Key highlights include:**

1. Fully Reproducible: Complete open-source software and hardware setup guaranteeing fully reproducible LIVO-RTK experiments.
2. Robust Initialization Module: Built-in initialization featuring DTW-based time offset estimation and hand-eye extrinsic calibration.
3. LIV-RTK Fusion Paradigm: A comprehensive example paradigm for fusing LIVO trajectories with RTK observations.

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

<div align="center">
  <img src="docs/images/compare.jpg" width="100%" alt="FAST-LIVO2 and FAST-LIVO2-RTK comparison in a degraded LiDAR-visual scene">
  <br>
  <sub>Collected in a challenging scene with degraded geometry and texture.
<b>a2, b2, c2</b>: FAST-LIVO2 baseline. <b>a1, b1, c1</b>: results after RTK fusion.</sub>
</div>

## 1. Prerequisited

### 1.1 Ubuntu and ROS

Ubuntu 18.04~20.04. See [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 1.2 PCL, Eigen, and OpenCV

PCL>=1.8, Eigen>=3.3.4, OpenCV>=4.2.

### 1.3 Sophus

Install the non-templated/double-only version of Sophus.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 1.4 Vikit

Vikit provides the camera models and math utilities required by this project. Put it in your catkin workspace source folder.

```bash
cd ~/catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

### 1.5 RTK Dependencies

The RTK branch depends on the GNSS ROS message package used by the u-blox/GVINS toolchain. Put it in the same catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git
```

GTSAM is used for factor graph-based post-processing optimization.

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j$(nproc)
sudo make install
```

GeographicLib is used for converting geographic coordinates to local Cartesian coordinates.

```bash
sudo apt-get install libgeographic-dev ros-${ROS_DISTRO}-eigen-conversions
```

## 2. Run our examples

Download the provided RTK test rosbag file: [RTK-Dataset](https://drive.google.com/file/d/1RIRcqjaw3x8l-S-Dc655xHi_bKkI7q66/view?usp=sharing).

1. Launch the system and load the configuration file:

```bash
roslaunch fast_livo HH.launch
```

2. Play the rosbag. Once the sequence is finished, press `Enter` in the terminal running the launch file to trigger the backend optimizer.

```bash
rosbag play HH-LVGO-01.bag
```
## 3. Appendix
**Time Synchronization:**

The diagram illustrates the time synchronization scheme among GNSS, LiDAR, and image data.

<div align="center">
  <img src="docs/images/sync.jpg" width="50%" alt="Time synchronization diagram">
</div>

**Hardware Platform:**

The table below summarizes the main devices used by the platform.

<div align="center">
  <table align="center">
    <tr>
      <th>Device</th>
      <th>Image</th>
      <th>Model Description</th>
    </tr>
    <tr>
      <td align="center">LiDAR</td>
      <td align="center"><img src="docs/images/LiDAR.jpg" width="180" alt="Livox Mid-360 LiDAR"></td>
      <td align="center">Model: Livox Mid-360</td>
    </tr>
    <tr>
      <td align="center">Camera</td>
      <td align="center"><img src="docs/images/camera.jpg" width="180" alt="MV-CB016-10GC-S-W camera"></td>
      <td align="center">Model: MV-CB016-10GC-S-W</td>
    </tr>
    <tr>
      <td align="center">GNSS Receiver</td>
      <td align="center"><img src="docs/images/ublox.jpg" width="180" alt="u-blox ZED-F9P GNSS receiver"></td>
      <td align="center">Model: u-blox ZED-F9P</td>
    </tr>
    <tr>
      <td align="center">Computing Unit</td>
      <td align="center"><img src="docs/images/n100.jpg" width="180" alt="N100 mini PC"></td>
      <td align="center">Model: N100 mini PC</td>
    </tr>
    <tr>
      <td align="center">Synchronization Controller</td>
      <td align="center"><img src="docs/images/stm32.jpg" width="180" alt="STM32 synchronization controller"></td>
      <td align="center">Model: STM32</td>
    </tr>
  </table>
</div>

## 4. Troubleshooting

> Notes from debugging on Ubuntu (ROS Noetic), updated 2026-06-13.

### 4.1 Segfault at startup in `cv::initUndistortRectifyMap` (OpenCV version conflict)

**Symptom:** the node crashes with `SIGSEGV` right after `scale: ...` is printed, inside
`vk::PinholeCamera::PinholeCamera` → `cv::initUndistortRectifyMap`. The camera parameters are
fine; it crashes with any valid config.

**Root cause:** two different OpenCV runtimes are loaded into the same process:

- ROS `cv_bridge` (the apt package in `/opt/ros/noetic`) is linked against the **system OpenCV 4.5.4** (`/usr/lib/x86_64-linux-gnu/libopencv_*.so.4.5d`);
- `rpg_vikit` / FAST-LIVO are linked against a **locally built OpenCV 4.6.0** (`/usr/local/lib/libopencv_*.so.406`).

The `cv::Mat` objects are created by 4.6.0 code, but `cv::initUndistortRectifyMap` resolves to the
4.5.4 copy loaded earlier via `cv_bridge`. Mixing two OpenCV ABIs in one process corrupts memory → crash.

**Fix — make the whole node use one OpenCV.** Build `cv_bridge` from source against the *same*
OpenCV that vikit uses (4.6.0), so the apt 4.5.4 `cv_bridge` is no longer pulled in:

```bash
cd ~/catkin_ws/src
git clone -b noetic https://github.com/ros-perception/vision_opencv.git   # already present here as vision_opencv-noetic
cd ~/catkin_ws
rm -rf build/CMakeCache.txt                  # clear the stale cv_bridge_DIR that pointed at apt cv_bridge
catkin_make
source devel/setup.bash                      # workspace cv_bridge now shadows the apt one
```

**Verify** the node links a single OpenCV (only `*.so.406`, no `*.4.5d`):

```bash
ldd devel/lib/fast_livo/fastlivo_mapping | grep opencv
```

(Alternative: instead of rebuilding `cv_bridge`, pin vikit/FAST-LIVO to the system OpenCV 4.5.4 via
`set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)` before each `find_package(OpenCV)` and rebuild.)

### 4.2 `catkin_make` configure failures on a clean build

- **googletest:** `cmake_minimum_required ... Compatibility with CMake < 3.5 has been removed`.
  The bundled gtest is too old for recent CMake. Build with `-DCATKIN_ENABLE_TESTING=OFF`
  (or `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`).
- **Non-catkin packages:** `This workspace contains non-catkin packages ... without isolation`.
  ROS 2 / `ament_cmake` packages were found in `src/` (e.g. `pbox_node_dirve/src/serial`,
  `pbox_node_dirve/src/imu_msgs`). Add an empty `CATKIN_IGNORE` file to each so `catkin_make` skips them.

## 5. Acknowledgements

This repository is built on top of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) and uses several open-source libraries and packages, including [GTSAM](https://github.com/borglab/gtsam), [GeographicLib](https://geographiclib.sourceforge.io/), and [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm).
