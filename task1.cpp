/*
 * Project 4: Camera Calibration and Augmented Reality
 * Task 1: Detect and Extract Target Corners
 *
 * Controls:
 *   'q' / ESC - quit
 */

#include <opencv2/opencv.hpp>
#include <iostream>

// Chessboard: 9 columns x 6 rows of internal corners
const cv::Size BOARD_SIZE(9, 6);

int main(int argc, char* argv[])
{
    // Open camera or video file
    cv::VideoCapture cap;
    if (argc > 1) cap.open(argv[1]);
    else          cap.open(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera.\n";
        return -1;
    }

    std::cout << "=== Task 1: Detect and Extract Target Corners ===\n"
              << "  Target: Chessboard " << BOARD_SIZE.width
              << " x " << BOARD_SIZE.height << " internal corners\n"
              << "  Press 'q' to quit\n\n";

    cv::Mat frame, gray;

    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to grayscale for detection
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ── Detect chessboard corners ──────────────────────────────────────
        std::vector<cv::Point2f> corner_set;

        bool found = cv::findChessboardCorners(
            gray, BOARD_SIZE, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (found) {
            // Refine corners to sub-pixel accuracy
            cv::cornerSubPix(
                gray, corner_set,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            // Draw detected corners
            cv::drawChessboardCorners(frame, BOARD_SIZE, corner_set, found);

            // Green border = detected
            cv::rectangle(frame, {0, 0}, {frame.cols-1, frame.rows-1}, {0,255,0}, 6);

            // Show corner count and first corner coordinates
            std::string info = "Corners: " + std::to_string(corner_set.size())
                + "  First: (" + std::to_string((int)corner_set[0].x)
                + ", "         + std::to_string((int)corner_set[0].y) + ")";
            cv::putText(frame, info, {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

            cv::putText(frame, "CHESSBOARD DETECTED", {10, frame.rows - 15},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);

            // Print to terminal
            std::cout << "Detected " << corner_set.size() << " corners. "
                      << "First corner: (" << corner_set[0].x
                      << ", " << corner_set[0].y << ")\n";
        } else {
            // Red border = not detected
            cv::rectangle(frame, {0, 0}, {frame.cols-1, frame.rows-1}, {0,0,255}, 6);
            cv::putText(frame, "No chessboard detected - show full board", {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
        }

        cv::imshow("Task 1 - Corner Detection", frame);

        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}