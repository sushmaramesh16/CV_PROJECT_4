/*
 * Project 4: Camera Calibration and Augmented Reality
 * Task 2: Select Calibration Images
 *
 * Builds on Task 1 by letting the user save frames for calibration.
 * Saves both the 2D corner locations and the 3D world points.
 *
 * Controls:
 *   's' - save current frame (only if corners are detected)
 *   'q' / ESC - quit
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// Chessboard: 9 columns x 6 rows of internal corners
const cv::Size BOARD_SIZE(9, 6);
const int      MIN_CALIB_FRAMES = 5;

// ─── Build 3-D world points for one chessboard pose ───────────────────────────
// Squares are 1x1 units. Origin at upper-left internal corner.
// X → right, Y → up (negated row), Z faces viewer.
std::vector<cv::Vec3f> buildObjectPoints()
{
    std::vector<cv::Vec3f> pts;
    pts.reserve(BOARD_SIZE.width * BOARD_SIZE.height);
    for (int r = 0; r < BOARD_SIZE.height; ++r)
        for (int c = 0; c < BOARD_SIZE.width; ++c)
            pts.emplace_back((float)c, -(float)r, 0.f);
    return pts;
}

int main(int argc, char* argv[])
{
    cv::VideoCapture cap;
    if (argc > 1) cap.open(argv[1]);
    else          cap.open(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera.\n";
        return -1;
    }

    std::cout << "=== Task 2: Select Calibration Images ===\n"
              << "  's' – save frame    'q' – quit\n"
              << "  Need at least " << MIN_CALIB_FRAMES << " frames\n\n";

    // ── Storage for calibration data ───────────────────────────────────────
    std::vector<std::vector<cv::Point2f>> corner_list;  // 2D image corners
    std::vector<std::vector<cv::Vec3f>>   point_list;   // 3D world points

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_set;
    bool cornersFound = false;

    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ── Task 1: Detect corners ─────────────────────────────────────────
        cornersFound = cv::findChessboardCorners(
            gray, BOARD_SIZE, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (cornersFound) {
            cv::cornerSubPix(
                gray, corner_set,
                cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            cv::drawChessboardCorners(frame, BOARD_SIZE, corner_set, cornersFound);
            cv::rectangle(frame, {0,0}, {frame.cols-1, frame.rows-1}, {0,255,0}, 6);

            std::string info = "Corners: " + std::to_string(corner_set.size())
                + "  First: (" + std::to_string((int)corner_set[0].x)
                + ", "         + std::to_string((int)corner_set[0].y) + ")";
            cv::putText(frame, info, {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

            cv::putText(frame, "Press 's' to save this frame",
                {10, frame.rows - 15},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
        } else {
            cv::rectangle(frame, {0,0}, {frame.cols-1, frame.rows-1}, {0,0,255}, 6);
            cv::putText(frame, "No chessboard detected - show full board", {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        }

        // ── Status overlay ─────────────────────────────────────────────────
        std::string status = "Saved: " + std::to_string(corner_list.size())
            + "/" + std::to_string(MIN_CALIB_FRAMES);
        cv::putText(frame, status, {10, 70},
            cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 200, 0}, 2);

        cv::imshow("Task 2 - Save Calibration Frames", frame);

        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27) break;

        // ── Task 2: Save frame ─────────────────────────────────────────────
        else if (key == 's') {
            if (!cornersFound) {
                std::cout << "No corners found - frame NOT saved.\n";
            } else {
                std::vector<cv::Vec3f> point_set = buildObjectPoints();
                corner_list.push_back(corner_set);
                point_list.push_back(point_set);

                int n = (int)corner_list.size();
                std::string fname = "calib_frame_" + std::to_string(n) + ".png";
                cv::imwrite(fname, gray);  // save clean grayscale image

                std::cout << "Frame " << n << " saved. "
                          << "Corners: " << corner_set.size()
                          << "  First 2D: (" << corner_set[0].x << ", " << corner_set[0].y << ")"
                          << "  First 3D: (" << point_set[0][0] << ", "
                          << point_set[0][1] << ", " << point_set[0][2] << ")\n"
                          << "  Image saved: " << fname << "\n";

                if (n >= MIN_CALIB_FRAMES)
                    std::cout << "  You have enough frames! Run task3 to calibrate.\n";
            }
        }
    }

    // Print summary
    std::cout << "\n=== Summary ===\n"
              << "Total frames saved: " << corner_list.size() << "\n";
    if ((int)corner_list.size() >= MIN_CALIB_FRAMES)
        std::cout << "Ready for calibration! Run task3.\n";
    else
        std::cout << "Need " << MIN_CALIB_FRAMES - corner_list.size()
                  << " more frames.\n";

    cap.release();
    cv::destroyAllWindows();
    return 0;
}