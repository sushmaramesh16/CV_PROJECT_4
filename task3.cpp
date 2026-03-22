/*
 * Name: Sushma Ramesh, Dina Barua
 * Date: March 22, 2026
 * Purpose: Task 3 - Calibrate the camera using saved calibration frames.
 *          Computes camera matrix and distortion coefficients via
 *          cv::calibrateCamera and saves intrinsic parameters to file.
 *
 * Builds on Tasks 1 & 2. Loads previously saved calib_frame_N.png files
 * automatically, then lets the user calibrate and save intrinsic parameters.
 *
 * Controls:
 *   's' - save current frame for calibration
 *   'c' - run calibration (requires >= 5 frames)
 *   'w' - write intrinsic parameters to intrinsic_params.yml
 *   'q' / ESC - quit
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <iomanip>

const cv::Size    BOARD_SIZE(9, 6);
const int         MIN_CALIB_FRAMES = 5;
const std::string INTRINSIC_FILE   = "intrinsic_params.yml";

// ─── Build 3-D world points ───────────────────────────────────────────────────
std::vector<cv::Vec3f> buildObjectPoints()
{
    std::vector<cv::Vec3f> pts;
    for (int r = 0; r < BOARD_SIZE.height; ++r)
        for (int c = 0; c < BOARD_SIZE.width; ++c)
            pts.emplace_back((float)c, -(float)r, 0.f);
    return pts;
}

// ─── Print a matrix neatly ────────────────────────────────────────────────────
void printMat(const std::string& label, const cv::Mat& M)
{
    std::cout << "\n" << label << ":\n";
    for (int r = 0; r < M.rows; ++r) {
        std::cout << "  [";
        for (int c = 0; c < M.cols; ++c) {
            std::cout << std::setw(14) << std::fixed << std::setprecision(6)
                      << M.at<double>(r, c);
            if (c < M.cols - 1) std::cout << ",";
        }
        std::cout << " ]\n";
    }
}

// ─── Load previously saved calibration frames ────────────────────────────────
int loadSavedFrames(
    std::vector<std::vector<cv::Point2f>>& corner_list,
    std::vector<std::vector<cv::Vec3f>>&   point_list)
{
    int loaded = 0;
    for (int i = 1; i <= 100; ++i) {
        std::string fname = "calib_frame_" + std::to_string(i) + ".png";
        cv::Mat img = cv::imread(fname);
        if (img.empty()) break;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, BOARD_SIZE, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));
            corner_list.push_back(corners);
            point_list.push_back(buildObjectPoints());
            std::cout << "  Loaded: " << fname
                      << "  (" << corners.size() << " corners)\n";
            ++loaded;
        } else {
            std::cout << "  Skipped (no corners): " << fname << "\n";
        }
    }
    return loaded;
}

// ─── Run calibration ─────────────────────────────────────────────────────────
double runCalibration(
    const std::vector<std::vector<cv::Point2f>>& corner_list,
    const std::vector<std::vector<cv::Vec3f>>&   point_list,
    cv::Size imageSize,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs)
{
    std::cout << "\n--- Camera matrix BEFORE calibration ---";
    printMat("camera_matrix", camera_matrix);

    double rpe = cv::calibrateCamera(
        point_list, corner_list, imageSize,
        camera_matrix, dist_coeffs,
        rvecs, tvecs,
        cv::CALIB_FIX_ASPECT_RATIO);

    std::cout << "\n--- Camera matrix AFTER calibration ---";
    printMat("camera_matrix", camera_matrix);
    printMat("dist_coeffs  ", dist_coeffs);
    std::cout << "\nRe-projection error: " << std::fixed
              << std::setprecision(4) << rpe << " pixels\n";

    return rpe;
}

// ─── Write intrinsics to file ─────────────────────────────────────────────────
void writeIntrinsics(const cv::Mat& cam, const cv::Mat& dist)
{
    cv::FileStorage fs(INTRINSIC_FILE, cv::FileStorage::WRITE);
    fs << "camera_matrix"     << cam;
    fs << "dist_coefficients" << dist;
    fs.release();
    std::cout << "Saved intrinsic parameters to: " << INTRINSIC_FILE << "\n";
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    cv::VideoCapture cap;
    if (argc > 1) cap.open(argv[1]);
    else          cap.open(0);

    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera.\n";
        return -1;
    }

    // ── Calibration data ───────────────────────────────────────────────────
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>>   point_list;
    std::vector<cv::Mat>                  rvecs, tvecs;

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_set;
    bool   cornersFound = false;
    bool   calibrated   = false;
    double lastRPE      = 0.0;

    // Default camera matrix (refined during calibration)
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    std::cout << "=== Task 3: Camera Calibration ===\n"
              << "  's' – save    'c' – calibrate    'w' – write params    'q' – quit\n\n";

    // ── Auto-load existing frames ──────────────────────────────────────────
    std::cout << "Scanning for previously saved calibration frames...\n";
    int loaded = loadSavedFrames(corner_list, point_list);
    if (loaded > 0)
        std::cout << "\nLoaded " << loaded << " frame(s). "
                  << "Press 'c' to calibrate immediately!\n\n";
    else
        std::cout << "No saved frames found. Show checkerboard and press 's'.\n\n";

    // ── Video loop ─────────────────────────────────────────────────────────
    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect corners (Task 1)
        cornersFound = cv::findChessboardCorners(
            gray, BOARD_SIZE, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (cornersFound) {
            cv::cornerSubPix(
                gray, corner_set,
                cv::Size(11,11), cv::Size(-1,-1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            cv::drawChessboardCorners(frame, BOARD_SIZE, corner_set, cornersFound);
            cv::rectangle(frame, {0,0}, {frame.cols-1, frame.rows-1}, {0,255,0}, 6);

            std::string info = "Corners: " + std::to_string(corner_set.size())
                + "  First: (" + std::to_string((int)corner_set[0].x)
                + ", "         + std::to_string((int)corner_set[0].y) + ")";
            cv::putText(frame, info, {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0,255,0}, 2);

            cv::putText(frame, "Press 's' to save!",
                {10, frame.rows - 15},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0,255,255}, 2);
        } else {
            cv::rectangle(frame, {0,0}, {frame.cols-1, frame.rows-1}, {0,0,255}, 6);
            cv::putText(frame, "No chessboard - show full board", {10, 35},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0,0,255}, 2);
        }

        // Status
        std::string status = "Saved: " + std::to_string(corner_list.size())
            + "/" + std::to_string(MIN_CALIB_FRAMES);
        cv::putText(frame, status, {10, 65},
            cv::FONT_HERSHEY_SIMPLEX, 0.65, {255,200,0}, 2);

        if (calibrated) {
            std::string rpe_str = "RPE: " + std::to_string(lastRPE).substr(0,6) + " px";
            cv::putText(frame, rpe_str, {10, 95},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0,200,255}, 2);
        }

        cv::imshow("Task 3 - Camera Calibration", frame);

        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27) break;

        // Save frame (Task 2)
        else if (key == 's') {
            if (!cornersFound) {
                std::cout << "No corners found - NOT saved.\n";
            } else {
                corner_list.push_back(corner_set);
                point_list.push_back(buildObjectPoints());
                int n = (int)corner_list.size();
                std::string fname = "calib_frame_" + std::to_string(n) + ".png";
                cv::imwrite(fname, frame);
                std::cout << "Frame " << n << " saved -> " << fname << "\n";
            }
        }

        // Calibrate (Task 3)
        else if (key == 'c') {
            if ((int)corner_list.size() < MIN_CALIB_FRAMES) {
                std::cout << "Need " << MIN_CALIB_FRAMES << " frames, have "
                          << corner_list.size() << ".\n";
            } else {
                camera_matrix = (cv::Mat_<double>(3,3) <<
                    1,           0, frame.cols / 2.0,
                    0,           1, frame.rows / 2.0,
                    0,           0, 1);

                lastRPE = runCalibration(
                    corner_list, point_list, frame.size(),
                    camera_matrix, dist_coeffs, rvecs, tvecs);
                calibrated = true;
            }
        }

        // Write intrinsics
        else if (key == 'w') {
            if (!calibrated)
                std::cout << "Run calibration ('c') first.\n";
            else
                writeIntrinsics(camera_matrix, dist_coeffs);
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}