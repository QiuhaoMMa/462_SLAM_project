# 📷 Monocular Visual SLAM in MATLAB

This repository contains the code, data, and report for a Monocular Visual SLAM (vSLAM) project completed as part of ME/ECE/CS 462 – Robotic Vision at SIUE. The project demonstrates how to perform camera trajectory estimation and 3D scene reconstruction using both a benchmark dataset (TUM RGB-D) and a custom video recording.

## 📌 Project Overview

Visual SLAM allows a system to simultaneously estimate its location and build a map of the environment using a monocular camera. This project includes:

- A MATLAB implementation of monocular SLAM based on the [MathWorks vSLAM example](https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html)
- Trajectory and point cloud reconstruction using the TUM RGB-D dataset
- A custom SLAM pipeline using a self-recorded indoor video


---

## 🛠 Requirements

- MATLAB R2023a or later
- Computer Vision Toolbox
- Image Processing Toolbox
- Camera Calibrator App (for intrinsic calibration)

