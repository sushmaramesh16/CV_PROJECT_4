# Project 4: Camera Calibration and Augmented Reality
CS 5330 - Computer Vision and Pattern Recognition
Northeastern University

## Author
Sushma Reddy

## Overview
This project implements camera calibration using a checkerboard target
and applies the calibration for augmented reality overlays.

## Files
- `task1.cpp` - Detect and extract chessboard corners
- `task2.cpp` - Select and save calibration images
- `task3.cpp` - Calibrate the camera
- `extension5.cpp` - Static images with AR overlay
- `extension6.cpp` - Make target unrecognizable with mosaic

## Build
```bash
cmake -S . -B build
cmake --build build
```

## Run
```bash
./build/task1        # Corner detection
./build/task2        # Save calibration frames
./build/task3        # Calibrate camera
./build/extension5   # Static AR overlay
./build/extension6   # Mosaic overlay
```

## Dependencies
- OpenCV 4.x
- CMake 3.10+
