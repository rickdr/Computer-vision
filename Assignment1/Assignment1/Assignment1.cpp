// CVAssignment1.cpp : This file contains the 'main' function. Program execution begins and ends there.
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include "settings.cpp"

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {

    // Read the settings from default.xml
    Settings s;
    CommandLineParser parser = s.initParser(argc, argv);
    const string inputSettingsFile = parser.get<string>(0);
    FileStorage fs(inputSettingsFile, FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();

    // Initialization of variables
    Mat image, gray_image;
    int image_amount;
    Size board_size = Size(s.boardSize);
    Mat overall_dist_coeffs, overall_intrinsic = Mat(3, 3, CV_32FC1);
    overall_intrinsic.ptr<float>(0)[0] = 1;
    overall_intrinsic.ptr<float>(1)[1] = 1;
    vector<float> overall_errors;
    bool done = false;
    float best_rms = 10;
    int loop = 0;

    // Get the image list from the settings file
    vector<String> images = s.imageList;

    // While loop: iterates over all images leaving out image (1 + iteration number)
    while (done == false and loop < images.size()) {
        std::cout << "Image set:" << loop << "\n";

        // Initialize lists of lists for image and object points
        vector<vector<Point3f>> object_points;
        vector<vector<Point2f>> image_points;
        vector<Point3f> objects;
        Size image_size;

        // Create the points on the board in 3d space, with the origin on the corner of square (0,0)
        for (int i = 0; i < board_size.height; i++)
            for (int j = 0; j < board_size.width; j++)
                objects.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));

        // Get the filtered image list from the settings file, and leave one out every iteration, except the first run
        vector<String> subset_images = images;
        if (loop > 0) {
            subset_images.erase(subset_images.begin() + loop);
        }
        cout << "Image set size:" << subset_images.size() << "\n";

        // Attempt to find the chessboard corners for each input image set.
        for (int image_amount = 0; image_amount < subset_images.size(); image_amount++) {
            cout << "\r" << "Processing image: " << image_amount + 1;
            image = imread(subset_images[image_amount], IMREAD_COLOR);
            image_size = image.size();

            // Gray scale the image
            cvtColor(image, gray_image, COLOR_BGR2GRAY);

            // Find the chessboard-patttern corners
            vector<Point2f> image_corners;
            bool found_chessboard = findChessboardCorners(gray_image, board_size, image_corners);

            if (found_chessboard) {
                // Improve the found corners
                cornerSubPix(gray_image, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

                // Draw the found chessboard corners
                drawChessboardCorners(image, board_size, Mat(image_corners), found_chessboard);

                // Store the results
                image_points.push_back(image_corners);
                object_points.push_back(objects);
            }

            if (waitKey(10) == 27) {
                break;
            }
        }

        Mat intrinsic = overall_intrinsic.clone(), dist_coeffs = overall_dist_coeffs.clone();
        vector<Mat> rvecs, tvecs;

        // Run calibration with the parameters found in the current set and update high scores accordingly.
        float rms = calibrateCamera(object_points, image_points, image_size, intrinsic, dist_coeffs, rvecs, tvecs);

        // When the altered set has a significant better rms we replace the current best, and save the variables of this set
        if (abs(best_rms - rms) > 0.01) {
            overall_intrinsic = intrinsic;
            overall_dist_coeffs = dist_coeffs;
            best_rms = rms;
            images = subset_images;
        }
        loop++;
        cout << "\r" << "Root mean square: " << rms << "\n";
    }
    cout << "Calibration completed." << "\n";
    cout << "Best rms: " << best_rms << "\n";
    cout << "Final intrinsic: " << overall_intrinsic << "\n";
    cout << "Final dist coeffs: " << overall_dist_coeffs << "\n";

    // Video Capture
    VideoCapture camera(1);
    if (!camera.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    // Create windows to display the images from the webcam before and after modification
    namedWindow("Webcam", WINDOW_AUTOSIZE);
    namedWindow("Undistorted", WINDOW_AUTOSIZE);

    // capture frame by frame from the webcam
    Mat frame, frame_gray, intrinsic, dist_coeffs, rvecs, tvecs, frame_undistorted;
    bool running = true;
    camera >> frame;
    while (running) {
        vector<vector<Point3f>> object_points;
        vector<Point3f> objects, cube_points_background, cube_points;
        vector<Point2f> frame_points, frame_points_background, frame_corners;

        // Apply the intrinsic matrix to the image
        undistort(frame, frame_undistorted, overall_intrinsic, overall_dist_coeffs);

        // Make the undistorted image grey 
        cvtColor(frame_undistorted, frame_gray, COLOR_BGR2GRAY);

        // Create the points on the board in 3d space, with the origin on the corner of square (0,0)
        for (int i = 0; i < board_size.height; i++) {
            for (int j = 0; j < board_size.width; j++) {
                objects.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));
            }
        }
        // Create the cube coordinates
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cube_points_background.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));
                cube_points.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, (-(s.squareSize * 2))));
            }
        }

        // Find the corners of the chessboard
        bool found_chessboard = findChessboardCorners(frame_gray, board_size, frame_corners);
        if (found_chessboard) {
            // Improve the chessboard corners
            solvePnP(objects, Mat(frame_corners), overall_intrinsic, dist_coeffs, rvecs, tvecs);

            // Project the points
            projectPoints(cube_points_background, rvecs, tvecs, overall_intrinsic, dist_coeffs, frame_points_background);
            projectPoints(cube_points, rvecs, tvecs, overall_intrinsic, dist_coeffs, frame_points);

            // Possible line colors
            vector<Scalar> colors = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };
            Scalar color = colors[rand() % colors.size()], color_background = colors[rand() % colors.size()], color_dimension = colors[rand() % colors.size()];

            // First draw the background
            line(frame_undistorted, frame_points_background[0], frame_points_background[2], color_background, 3);
            line(frame_undistorted, frame_points_background[2], frame_points_background[8], color_background, 3);
            line(frame_undistorted, frame_points_background[6], frame_points_background[0], color_background, 3);
            line(frame_undistorted, frame_points_background[8], frame_points_background[6], color_background, 3);

            // Then draw the pillars
            line(frame_undistorted, frame_points[0], frame_points_background[0], color, 3);
            line(frame_undistorted, frame_points[2], frame_points_background[2], color, 3);
            line(frame_undistorted, frame_points[6], frame_points_background[6], color, 3);
            line(frame_undistorted, frame_points[8], frame_points_background[8], color, 3);

            // Next the lines in dimension
            line(frame_undistorted, frame_points[0], frame_points[2], color_dimension, 3);
            line(frame_undistorted, frame_points[2], frame_points[8], color_dimension, 3);
            line(frame_undistorted, frame_points[6], frame_points[0], color_dimension, 3);
            line(frame_undistorted, frame_points[8], frame_points[6], color_dimension, 3);
        }

        imshow("Webcam", frame);
        imshow("Undistorted", frame_undistorted);
        camera >> frame;

        if (waitKey(10) == 27) {
            destroyWindow("Webcam");
            destroyWindow("Undistorted");
            running = false;
        }
    }
    camera.release();
    return 0;
}