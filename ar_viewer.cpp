/*
 * ar_viewer.cpp
 * Name: Dina Barua, Sushma Ramesh
 * Date: March 21,2026
 * Tasks 4, 5, 6, 7
 *
 * Purpose: One single program that covers all four of my tasks:
 *
 *   Task 4 - reads the calibration my partner saved, detects the
 *            chessboard every frame, and uses solvePnP to figure
 *            out where the camera is relative to the board
 *
 *   Task 5 - draws X/Y/Z axes on the board using projectPoints
 *
 *   Task 6 - draws a wireframe house floating above the board
 *
 *   Task 7 - shows Harris corners OR ORB features on the live feed
 *            with trackbars to adjust settings
 *
 * Controls:
 *   'n'  - normal AR mode (tasks 4/5/6) - this is the default
 *   'h'  - switch to Harris corner mode (task 7)
 *   'r'  - switch to ORB feature mode (task 7)
 *   'a'  - toggle axes on/off (works in normal mode)
 *   'o'  - toggle house on/off (works in normal mode)
 *   's'  - save a screenshot
 *   'q'  - quit
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// the chessboard has 9 corner columns and 6 corner rows
// (I kept mixing these up so I put them here at the top)
const int BOARD_COLS = 9;
const int BOARD_ROWS = 6;

// trackbar variables for Harris (task 7)
// these are global so the trackbar callback can reach them
int harrisThresh    = 100;  // 0-255 response threshold
int harrisBlockSize = 2;    // neighborhood size 2-9

// -------------------------------------------------------------------------
// Load the camera matrix and distortion values that my partner saved
// in task 3 - the file is called intrinsic_params.yml
// -------------------------------------------------------------------------
bool loadCalibration(const std::string &filename,
                     cv::Mat &cameraMatrix,
                     cv::Mat &distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERROR: could not open " << filename << "\n";
        return false;
    }
    fs["camera_matrix"]           >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // print them out so I can confirm they loaded correctly
    std::cout << "Loaded camera matrix:\n"    << cameraMatrix << "\n\n";
    std::cout << "Loaded distortion coeffs:\n" << distCoeffs  << "\n\n";
    return true;
}

// -------------------------------------------------------------------------
// Build the 3D world point grid for the chessboard
// Each square = 1 unit.  Top-left corner = (0, 0, 0)
// X goes right, Y goes up the board (so rows are negative), Z = 0
// -------------------------------------------------------------------------
std::vector<cv::Point3f> buildWorldPoints()
{
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < BOARD_ROWS; r++)
        for (int c = 0; c < BOARD_COLS; c++)
            pts.push_back(cv::Point3f((float)c, -(float)r, 0.0f));
    return pts;
}

// =========================================================================
// TASK 5 -- Draw 3D axes on the board
//
// We project 4 points (origin + one tip per axis) into image space
// using projectPoints, then draw colored lines between them.
// Red = X, Green = Y, Blue = Z  (same as the spec image)
// =========================================================================
void drawAxes(cv::Mat &frame,
              const cv::Mat &K, const cv::Mat &D,
              const cv::Mat &rvec, const cv::Mat &tvec,
              float len = 3.0f)
{
    std::vector<cv::Point3f> pts3D;
    pts3D.push_back(cv::Point3f(0,    0,    0));    // origin
    pts3D.push_back(cv::Point3f(len,  0,    0));    // +X tip
    pts3D.push_back(cv::Point3f(0,   -len,  0));    // +Y tip
    pts3D.push_back(cv::Point3f(0,    0,   -len));  // -Z tip (pokes toward viewer)

    // projectPoints converts 3D world coords to 2D image pixels
    std::vector<cv::Point2f> pts2D;
    cv::projectPoints(pts3D, rvec, tvec, K, D, pts2D);

    // draw a colored line from the origin to each axis tip
    cv::line(frame, pts2D[0], pts2D[1], cv::Scalar(0,   0,   255), 3); // X red
    cv::line(frame, pts2D[0], pts2D[2], cv::Scalar(0,   255, 0),   3); // Y green
    cv::line(frame, pts2D[0], pts2D[3], cv::Scalar(255, 0,   0),   3); // Z blue

    // small labels so you can tell which axis is which
    cv::putText(frame, "X", pts2D[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,   0,   255), 2);
    cv::putText(frame, "Y", pts2D[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,   255, 0),   2);
    cv::putText(frame, "Z", pts2D[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0,   0),   2);
}

// =========================================================================
// TASK 6 -- Draw a wireframe house floating above the board
//
// I designed the house on paper first as a list of 3D line endpoints,
// then used projectPoints to turn them into 2D screen positions.
//
// Three colors: teal = walls, orange = roof, white = details
// The chimney and door are off-center on purpose - makes it obvious if the orientation is wrong while debugging
// =========================================================================
void drawHouse(cv::Mat &frame,
               const cv::Mat &K, const cv::Mat &D,
               const cv::Mat &rvec, const cv::Mat &tvec)
{
    // center position and size of the house in board-square units
    float cx  = 4.0f;
    float cy  = -2.5f;
    float hw  = 1.5f;   // half width  (X direction)
    float hd  = 1.0f;   // half depth  (Y direction)
    float top = -2.0f;  // Z height of wall tops (negative = above the board)

    std::vector<cv::Point3f> pts;

    // 0-3: bottom face corners, sitting on the board (Z = 0)
    pts.push_back(cv::Point3f(cx - hw, cy + hd, 0.0f));  //  0 back-left
    pts.push_back(cv::Point3f(cx + hw, cy + hd, 0.0f));  //  1 back-right
    pts.push_back(cv::Point3f(cx + hw, cy - hd, 0.0f));  //  2 front-right
    pts.push_back(cv::Point3f(cx - hw, cy - hd, 0.0f));  //  3 front-left

    // 4-7: top of the walls (same XY positions, raised to Z = top)
    pts.push_back(cv::Point3f(cx - hw, cy + hd, top));   //  4
    pts.push_back(cv::Point3f(cx + hw, cy + hd, top));   //  5
    pts.push_back(cv::Point3f(cx + hw, cy - hd, top));   //  6
    pts.push_back(cv::Point3f(cx - hw, cy - hd, top));   //  7

    // 8: single tip of the pyramid roof
    pts.push_back(cv::Point3f(cx, cy, top - 1.5f));      //  8

    // 9-12: chimney base (pushed toward back-right so it's not centered)
    pts.push_back(cv::Point3f(cx + 0.5f, cy + 0.3f, top - 0.4f));  //  9
    pts.push_back(cv::Point3f(cx + 1.0f, cy + 0.3f, top - 0.4f));  // 10
    pts.push_back(cv::Point3f(cx + 1.0f, cy + 0.7f, top - 0.4f));  // 11
    pts.push_back(cv::Point3f(cx + 0.5f, cy + 0.7f, top - 0.4f));  // 12

    // 13-16: chimney top cap
    pts.push_back(cv::Point3f(cx + 0.5f, cy + 0.3f, top - 1.1f));  // 13
    pts.push_back(cv::Point3f(cx + 1.0f, cy + 0.3f, top - 1.1f));  // 14
    pts.push_back(cv::Point3f(cx + 1.0f, cy + 0.7f, top - 1.1f));  // 15
    pts.push_back(cv::Point3f(cx + 0.5f, cy + 0.7f, top - 1.1f));  // 16

    // 17-20: window on the front face of the house
    pts.push_back(cv::Point3f(cx - 0.55f, cy - hd, top * 0.35f));  // 17 bot-left
    pts.push_back(cv::Point3f(cx + 0.55f, cy - hd, top * 0.35f));  // 18 bot-right
    pts.push_back(cv::Point3f(cx + 0.55f, cy - hd, top * 0.75f));  // 19 top-right
    pts.push_back(cv::Point3f(cx - 0.55f, cy - hd, top * 0.75f));  // 20 top-left

    // 21-24: door (shifted left so it's not in the center)
    pts.push_back(cv::Point3f(cx - 0.9f, cy - hd, 0.0f));          // 21 bot-left
    pts.push_back(cv::Point3f(cx - 0.3f, cy - hd, 0.0f));          // 22 bot-right
    pts.push_back(cv::Point3f(cx - 0.3f, cy - hd, top * 0.5f));    // 23 top-right
    pts.push_back(cv::Point3f(cx - 0.9f, cy - hd, top * 0.5f));    // 24 top-left

    // project all 3D points to 2D using the pose from solvePnP
    std::vector<cv::Point2f> p;
    cv::projectPoints(pts, rvec, tvec, K, D, p);

    cv::Scalar teal  (180, 150,  20);  // walls
    cv::Scalar orange(  0, 140, 255);  // roof
    cv::Scalar white (255, 255, 255);  // chimney, window, door
    int T = 2;

    // draw walls: bottom ring + top ring + 4 vertical edges
    for (int i = 0; i < 4; i++) cv::line(frame, p[i],    p[(i+1)%4],    teal,   T);
    for (int i = 0; i < 4; i++) cv::line(frame, p[4+i],  p[4+(i+1)%4], teal,   T);
    for (int i = 0; i < 4; i++) cv::line(frame, p[i],    p[i+4],        teal,   T);

    // draw pyramid roof (each top corner to the single apex)
    for (int i = 4; i < 8; i++) cv::line(frame, p[i], p[8], orange, T);

    // draw chimney: base ring + cap ring + 4 vertical sides
    for (int i = 0; i < 4; i++) cv::line(frame, p[9+i],  p[9+(i+1)%4],  white, T);
    for (int i = 0; i < 4; i++) cv::line(frame, p[13+i], p[13+(i+1)%4], white, T);
    for (int i = 0; i < 4; i++) cv::line(frame, p[9+i],  p[13+i],       white, T);

    // draw window outline + cross panes inside
    for (int i = 0; i < 4; i++) cv::line(frame, p[17+i], p[17+(i+1)%4], white, T);
    cv::line(frame, p[17], p[19], white, 1);
    cv::line(frame, p[18], p[20], white, 1);

    // draw door
    cv::line(frame, p[21], p[22], white, T);
    cv::line(frame, p[22], p[23], white, T);
    cv::line(frame, p[23], p[24], white, T);
    cv::line(frame, p[24], p[21], white, T);
}

// =========================================================================
// TASK 7 -- Harris corner detection
//
// cornerHarris gives every pixel a score.
// High score = strong corner.  Negative = edge.  Near zero = flat.
//
// I normalize the map, threshold it, then do non-maximum suppression
// (only keep the strongest pixel in each 3x3 block) so each real
// corner gets one circle instead of a messy cluster.
// =========================================================================
void showHarris(const cv::Mat &gray, cv::Mat &display)
{
    cv::Mat response;
    // blockSize = how big a neighborhood to look at
    // 3         = Sobel kernel size (must be 3, 5, or 7)
    // 0.04      = Harris k value (commonly between 0.04 and 0.06)
    cv::cornerHarris(gray, response, std::max(2, harrisBlockSize), 3, 0.04);

    // normalize to 0-255 so the threshold slider makes sense
    cv::Mat norm;
    cv::normalize(response, norm, 0, 255, cv::NORM_MINMAX, CV_32F);

    int count = 0;
    for (int r = 1; r < norm.rows - 1; r++) {
        for (int c = 1; c < norm.cols - 1; c++) {
            float val = norm.at<float>(r, c);
            if (val > (float)harrisThresh) {
                // only draw if this is the local max in a 3x3 area
                bool isMax = true;
                for (int dr = -1; dr <= 1 && isMax; dr++)
                    for (int dc = -1; dc <= 1 && isMax; dc++)
                        if (norm.at<float>(r+dr, c+dc) > val)
                            isMax = false;
                if (isMax) {
                    cv::circle(display, cv::Point(c, r), 5,
                               cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                    count++;
                }
            }
        }
    }

    cv::putText(display, "Task 7: HARRIS  corners: " + std::to_string(count),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 255), 2);
    cv::putText(display, "Press N for AR mode  |  R for ORB",
                cv::Point(10, display.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
}

// =========================================================================
// TASK 7 -- ORB feature detection
//
// ORB = Oriented FAST + Rotated BRIEF.
// It finds corners and describes them with a binary descriptor.
// Unlike Harris, ORB handles scale and rotation changes, which
// makes it better for matching features across different viewpoints.
//
// DRAW_RICH_KEYPOINTS draws each feature as a circle (scale = size)
// with a line showing its orientation.
// =========================================================================
void showORB(const cv::Mat &gray, cv::Mat &display)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500); // detect up to 500 keypoints

    std::vector<cv::KeyPoint> keypoints;
    orb->detect(gray, keypoints);

    // draw with scale and orientation visible
    cv::drawKeypoints(display, keypoints, display,
                      cv::Scalar(0, 200, 255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    cv::putText(display, "Task 7: ORB  keypoints: " + std::to_string((int)keypoints.size()),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 200, 255), 2);
    cv::putText(display, "Press N for AR mode  |  H for Harris",
                cv::Point(10, display.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
}

// =========================================================================
// main
// =========================================================================
int main(int argc, char *argv[])
{
    // default file name matches what my partner saved in task 3
    std::string calibFile = "intrinsic_params.yml";
    if (argc > 1)
        calibFile = argv[1];

    // load the calibration
    cv::Mat K, D;
    if (!loadCalibration(calibFile, K, D))
        return 1;

    // open the webcam
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: cannot open camera\n";
        return 1;
    }

    cv::Size boardSize(BOARD_COLS, BOARD_ROWS);
    std::vector<cv::Point3f> worldPts = buildWorldPoints();

    bool showAxes  = true;  // toggle for task 5
    bool showHouse = true;  // toggle for task 6
    char mode      = 'n';   // 'n' = normal AR, 'h' = Harris, 'r' = ORB
    int  frameNum  = 0;

    // create the window and add Harris trackbars for task 7
    std::string winName = "AR Viewer - Tasks 4/5/6/7";
    cv::namedWindow(winName);
    cv::createTrackbar("Harris Thresh",      winName, &harrisThresh,    255);
    cv::createTrackbar("Harris Block Size",  winName, &harrisBlockSize, 9);

    std::cout << "AR Viewer running\n";
    std::cout << "Keys:  n=AR mode  h=Harris  r=ORB  a=axes  o=house  s=save  q=quit\n\n";

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        frameNum++;

        // grayscale copy used for detection
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ── TASK 7 modes ─────────────────────────────────────────────────
        if (mode == 'h') {
            cv::Mat display = frame.clone();
            showHarris(gray, display);
            cv::imshow(winName, display);
        }
        else if (mode == 'r') {
            cv::Mat display = frame.clone();
            showORB(gray, display);
            cv::imshow(winName, display);
        }
        // ── NORMAL AR mode (tasks 4, 5, 6) ───────────────────────────────
        else {
            // try to find the chessboard in this frame
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, boardSize, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH |
                cv::CALIB_CB_NORMALIZE_IMAGE |
                cv::CALIB_CB_FAST_CHECK);

            if (found) {
                // refine corners to sub-pixel accuracy
                cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(
                        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                        30, 0.001));

                // TASK 4: get the board pose with solvePnP
                cv::Mat rvec, tvec;
                cv::solvePnP(worldPts, corners, K, D, rvec, tvec);

                // print every 15 frames so the console isn't too spammy
                if (frameNum % 15 == 0) {
                    std::cout << "rvec: " << rvec.t() << "\n";
                    std::cout << "tvec: " << tvec.t() << "\n\n";
                }

                // show detected corners on screen
                cv::drawChessboardCorners(frame, boardSize, corners, found);

                // TASK 5: axes
                if (showAxes)
                    drawAxes(frame, K, D, rvec, tvec);

                // TASK 6: house
                if (showHouse)
                    drawHouse(frame, K, D, rvec, tvec);

            } else {
                cv::putText(frame, "Looking for board...", cv::Point(15, 35),
                            cv::FONT_HERSHEY_SIMPLEX, 0.9,
                            cv::Scalar(0, 80, 255), 2);
            }

            // small status bar at the bottom
            std::string hud = std::string("Axes:") + (showAxes  ? "ON " : "OFF") +
                              "  House:" +           (showHouse ? "ON"  : "OFF") +
                              "  [H]=Harris  [R]=ORB";
            cv::putText(frame, hud, cv::Point(10, frame.rows - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(220, 220, 220), 1);

            cv::imshow(winName, frame);
        }

        // handle key presses
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if (key == 'n') mode = 'n';           // back to AR mode
        if (key == 'h') mode = 'h';           // Harris mode
        if (key == 'r') mode = 'r';           // ORB mode
        if (key == 'a') showAxes  = !showAxes;
        if (key == 'o') showHouse = !showHouse;
        if (key == 's') {
            cv::imwrite("ar_snapshot.png", frame);
            std::cout << "Saved ar_snapshot.png\n";
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
