# Project 4: Camera Calibration and Augmented Reality
CS 5330 - Computer Vision and Pattern Recognition
Northeastern University

## Group Members
- Sushma Ramesh
- Dina Barua

## GitHub Repository
https://github.com/sushmaramesh16/CV_PROJECT_4

## Project Structure
- `task1.cpp` - Detect and extract chessboard corners
- `task2.cpp` - Select and save calibration images
- `task3.cpp` - Calibrate the camera
- `extension5.cpp` - Static images with AR overlay
- `extension6.cpp` - Make target unrecognizable with mosaic
- `CMakeLists.txt` - Build configuration
- `intrinsic_params.yml` - Saved camera intrinsic parameters

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

## Videos
No videos submitted.

## Time Travel Days
Using 2 time travel days for this assignment.

## Dependencies
- OpenCV 4.x
- CMake 3.10+
