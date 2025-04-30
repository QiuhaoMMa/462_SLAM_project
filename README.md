# 📷 Monocular Visual SLAM in MATLAB

This repository contains the code, data, and report for a Monocular Visual SLAM (vSLAM) project completed as part of ME/ECE/CS 462 – Robotic Vision at SIUE. The project demonstrates how to perform camera trajectory estimation and 3D scene reconstruction using both a benchmark dataset (TUM RGB-D) and a custom video recording.

## 📌 Project Overview

Visual SLAM allows a system to simultaneously estimate its location and build a map of the environment using a monocular camera. This project includes:

- A MATLAB implementation of monocular SLAM based on the [MathWorks vSLAM example](https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html)
- Trajectory and point cloud reconstruction using the TUM RGB-D dataset
- A custom SLAM pipeline using a self-recorded indoor video

## 📁 Repository Structure

├── example_dataset/ # (Optional) TUM RGB-D image sequence ├── custom_video_frames/ # Grayscale frames extracted from custom video ├── results/ # Screenshots and result plots ├── report/ # Final report (PDF and LaTeX) ├── SLAM_TUM_example.m # vSLAM on benchmark dataset ├── SLAM_CustomVideo.m # vSLAM on self-recorded video ├── calibration_parameters.mat # Camera intrinsics ├── README.md # This file


---

## 🛠 Requirements

- MATLAB R2023a or later
- Computer Vision Toolbox
- Image Processing Toolbox
- Camera Calibrator App (for intrinsic calibration)

---

## 🚀 How to Run

### ▶ TUM Dataset (Benchmark)

1. Download the dataset from:  
   [https://vision.in.tum.de/data/datasets/rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)

2. Place the `rgb/` image sequence inside `example_dataset/`.

3. Run the provided script:

```matlab
SLAM_TUM_example.m
▶ Custom Video Input
Place your own .mp4 video in the working directory.

Modify and run:
