cmake_minimum_required(VERSION 3.10)
project(CalibrationAR)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Task 1: Detect and Extract Target Corners
add_executable(task1 task1.cpp)
target_link_libraries(task1 ${OpenCV_LIBS})

# Task 2: Select Calibration Images
add_executable(task2 task2.cpp)
target_link_libraries(task2 ${OpenCV_LIBS})

# Task 3: Calibrate the Camera
add_executable(task3 task3.cpp)
target_link_libraries(task3 ${OpenCV_LIBS})

# Extension 5: Static Images with AR Overlay
add_executable(extension5 extension5.cpp)
target_link_libraries(extension5 ${OpenCV_LIBS})

# Extension 6: Make Target Unrecognizable
add_executable(extension6 extension6.cpp)
target_link_libraries(extension6 ${OpenCV_LIBS})