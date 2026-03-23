# Name: Sushma Ramesh, Dina Barua
# Date: March 22, 2026
# Purpose: README file for Project 4 - Camera Calibration and Augmented Reality

---

## Authors
- Sushma Ramesh
- Dina Barua

---

## GitHub Repository
https://github.com/sushmaramesh16/CV_PROJECT_4

---

## Operating System & IDE
- **OS:** macOS
- **IDE:** VS Code
- **Language:** C++ 17
- **Libraries:** OpenCV 4.13.0, CMake 4.3.0

---

## Project Description
This project implements a camera calibration pipeline and augmented reality overlays
using OpenCV. A standard 9x6 checkerboard target is used to detect corners, save
calibration frames, and compute the camera's intrinsic parameters via
cv::calibrateCamera. The calibrated parameters are saved to a file and used to
apply augmented reality overlays on both live video and static images.

The full pipeline consists of:
1. Real-time chessboard corner detection using findChessboardCorners and cornerSubPix
2. Interactive calibration frame selection with 3D world point assignment
3. Camera calibration using calibrateCamera with CALIB_FIX_ASPECT_RATIO flag
4. Static image AR overlay using solvePnP and projectPoints (Extension 5)
5. Checkerboard target replacement with colorful mosaic pattern (Extension 6)

Calibration Results (MacBook Pro webcam):
- Focal length: fx = fy = 1388.72 px
- Principal point: (962.36, 559.46)
- Re-projection error: 0.13 pixels

---

## Dependencies
Install OpenCV and CMake via Homebrew before building:
```
brew install opencv cmake
```
- OpenCV version: 4.13.0
- CMake version: 4.3.0+
- C++ Standard: 17

---

## File Structure
```
CV_PROJECT4/
├── task1.cpp            # Task 1: Real-time chessboard corner detection
├── task2.cpp            # Task 2: Interactive calibration frame selection
├── task3.cpp            # Task 3: Camera calibration and intrinsic parameter saving
├── extension5.cpp       # Extension 5: Static images with AR overlay
├── extension6.cpp       # Extension 6: Make target unrecognizable with mosaic
├── CMakeLists.txt       # CMake build configuration
├── intrinsic_params.yml # Saved camera matrix and distortion coefficients
└── README.md            # This file
```

---

## Build Instructions
```bash
cmake -S . -B build
cmake --build build
```

---

## How to Run

### Task 1 - Detect and Extract Target Corners
Opens the webcam and detects chessboard corners in real time.
Green border = detected, Red border = not detected.
Prints corner count and first corner coordinates to terminal.
```bash
./build/task1              # live webcam
./build/task1 video.mov    # video file
```

### Task 2 - Select Calibration Images
Extends Task 1 by letting the user save frames for calibration.
Press 's' to save a frame, 'q' to quit.
Saves clean grayscale calib_frame_N.png files to disk.
```bash
./build/task2              # live webcam
./build/task2 video.mov    # video file
```
Controls:
- s : save current frame (only if corners detected)
- q : quit

### Task 3 - Calibrate the Camera
Auto-loads previously saved calib_frame_*.png files on startup.
Requires at least 5 frames. Prints camera matrix before and after.
Saves intrinsic parameters to intrinsic_params.yml.
```bash
./build/task3              # live webcam
./build/task3 video.mov    # video file
```
Controls:
- s : save additional frame
- c : run calibration (requires 5+ frames)
- w : write intrinsic parameters to intrinsic_params.yml
- q : quit

### Extension 5 - Static Images with AR Overlay
Loads all calib_frame_*.png files, detects corners, estimates pose
via solvePnP, and overlays 3D coordinate axes using projectPoints.
Output saved as ext5_output_*.png.
```bash
./build/extension5                  # process all calib_frame_*.png
./build/extension5 my_image.png     # process a single image
```

### Extension 6 - Make Target Unrecognizable
Replaces the checkerboard pattern with a colorful mosaic tile pattern
while preserving correct AR geometry with 3D axes on top.
Output saved as ext6_output_*.png.
```bash
./build/extension6                  # process all calib_frame_*.png
./build/extension6 my_image.png     # process a single image
```

---

## Extensions

### Extension 5 - Static Images with AR Overlay
Demonstrates that the AR pipeline works on static images rather than
only live video. Reads frames from disk using cv::imread instead of
VideoCapture. Loads intrinsic_params.yml, detects corners, solves pose
with solvePnP, and projects 3D axes and board outline onto each image.
Implemented in extension5.cpp.

### Extension 6 - Make Target Unrecognizable
Replaces the checkerboard visually with a colorful stained-glass mosaic
pattern. For each square on the board, the 4 corners are projected from
3D world space to 2D image space using projectPoints, then filled with
a unique HSV-based color using fillConvexPoly. 3D axes are drawn on top
to prove the AR geometry is still correct.
Implemented in extension6.cpp.

---

## Videos
No videos submitted.

## Time Travel Days
Using 2 time travel days for this assignment.