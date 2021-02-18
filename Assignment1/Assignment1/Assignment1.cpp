// opencv-1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "Settings.cpp"
//#include "calibration.cpp"
//#include <parser.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    //double debug_best_rms = 2.2111502248730317;
    //Mat debug_overall_dist_coeffs = Mat(1, 5, CV_32FC1, new double[0.08982088366543066, -0.3587634505363069, -0.0001607385585405284, 0.0006984187450869396, -0.5076361699022028]);

    //float vars[3][3] = { {1784.272994731757, 0, 945.319154612644}, {0, 1759.250305069632, 508.9901046170839}, {0, 0, 1 } };
    //Mat debug_overall_intrinsic = Mat(3, 3, CV_32FC1, vars);

    //// Read the settings from default.xml
    Settings s;
    CommandLineParser parser = s.initParser(argc, argv);
    const string inputSettingsFile = parser.get<string>(0);
    FileStorage fs(inputSettingsFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();

    //// Video Capture
    //VideoCapture debug_camera(1);
    //if (!debug_camera.isOpened()) {
    //    cerr << "ERROR: Could not open camera" << endl;
    //    return 1;
    //}

    //// Create windows to display the images from the webcam before and after modification
    //cv::namedWindow("Webcam", WINDOW_AUTOSIZE);
    //cv::namedWindow("Undistorted", WINDOW_AUTOSIZE);

    //// capture frame by frame from the webcam
    //Mat debug_frame;
    //Mat debug_imageUndistorted;
    //debug_camera >> debug_frame;
    //bool debug_running = true;
    //while (debug_running) {
    //    undistort(debug_frame, debug_imageUndistorted, debug_overall_intrinsic, debug_overall_dist_coeffs);

    //    imshow("Webcam", debug_frame);
    //    imshow("Undistorted", debug_imageUndistorted);

    //    debug_camera >> debug_frame;

    //    if (waitKey(10) == 27)
    //    {
    //        destroyWindow("Webcam");
    //        destroyWindow("Undistorted");
    //        debug_running = false;
    //    }
    //}
    //debug_camera.release();

    // Initialization of variables
    Mat image, gray_image;
    int image_amount;

    Size board_size = Size(s.boardSize);
    vector<vector<Point3f>> overall_object_points;
    vector<vector<Point2f>> overall_image_points;
    Size overall_image_size;
    Mat overall_dist_coeffs, overall_intrinsic = Mat(3, 3, CV_32FC1);
    vector<Mat> overall_rvecs, overall_tvecs;

    vector<float> overall_errors;

    overall_intrinsic.ptr<float>(0)[0] = 1;
    overall_intrinsic.ptr<float>(1)[1] = 1;

    bool done = false;
    float best_rms = 10;
    int loop = 0;

    // Get the image list from the settings file
    vector<String> images = s.imageList;

    // While loop: iterates over all images leaving out image (1 + iteration number)
    while (done == false and loop < images.size())
    {
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
        if (loop > 0)
        {
            subset_images.erase(subset_images.begin() + loop);
        }

        for (int image_amount = 0; image_amount < subset_images.size(); image_amount++)
        {
            //std::cout << "\r" << "Processing image: " << image_amount;
            image = imread(subset_images[image_amount], IMREAD_COLOR);
            image_size = image.size();

            vector<Point2f> image_corners;
            cvtColor(image, gray_image, COLOR_BGR2GRAY);
            bool found_chessboard = findChessboardCorners(gray_image, board_size, image_corners);

            if (found_chessboard)
            {
                cornerSubPix(gray_image, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
                drawChessboardCorners(image, board_size, Mat(image_corners), found_chessboard);

                image_points.push_back(image_corners);
                object_points.push_back(objects);
            }
            //namedWindow("Image display");
            //imshow("window", image);
            //waitKey(0);

            if (waitKey(10) == 27)
            {
                break;
            }
        }

        Mat intrinsic = overall_intrinsic.clone(), dist_coeffs = overall_dist_coeffs.clone();
        vector<Mat> rvecs, tvecs;

        // Run calibration with the parameters found in the current set and update high scores accordingly.
        float rms = calibrateCamera(object_points, image_points, image_size, intrinsic, dist_coeffs, rvecs, tvecs);
        if (abs(best_rms - rms) > 0.05)
        //if (best_rms > rms)
        {
            overall_object_points = object_points;
            overall_image_points = image_points;
            overall_image_size = image_size;
            overall_intrinsic = intrinsic;
            overall_dist_coeffs = dist_coeffs;
            overall_rvecs = rvecs;
            overall_tvecs = tvecs;
            best_rms = rms;
            images = subset_images;
            std::cout << "\r";

            //loop--;
        }

        loop++;

        std::cout << "\r" << "Root mean square: " << rms << "\n";
    }
    std::cout << "Calibration completed." << "\n";
    std::cout << "Best rms: " << best_rms << "\n";
    /*for (size_t i = 0; i < overall_object_points.size(); i++)
    {
        std::cout << "Final object points " << i << ": " << overall_object_points[i] << "\n";
    }
    for (size_t i = 0; i < overall_image_points.size(); i++)
    {
        std::cout << "Final image points: " << i << ": " << overall_image_points[i] << "\n";
    }
    std::cout << "Final image size: " << overall_image_size << "\n";*/
    std::cout << "Final intrinsic: " << overall_intrinsic << "\n";
    std::cout << "Final dist coeffs: " << overall_dist_coeffs << "\n";

    // Video Capture
    VideoCapture camera(1);
    if (!camera.isOpened()) {
        cerr << "ERROR: Could not open camera" << endl;
        return 1;
    }

    // Create windows to display the images from the webcam before and after modification
    cv::namedWindow("Webcam", WINDOW_AUTOSIZE);
    cv::namedWindow("Undistorted", WINDOW_AUTOSIZE);

    // capture frame by frame from the webcam
    Mat frame, intrinsic, dist_coeffs, rvecs, tvecs, frame_undistorted;
    bool running = true;
    camera >> frame;
    while (running) {
        vector<vector<Point3f>> object_points;
        vector<Point3f> objects, cube_points_background, cube_points;
        vector<Point2f> frame_points, frame_points_background, frame_corners;

        undistort(frame, frame_undistorted, overall_intrinsic, overall_dist_coeffs);
        cvtColor(frame_undistorted, gray_image, COLOR_BGR2GRAY);

        // Create the points on the board in 3d space, with the origin on the corner of square (0,0)
        for (int i = 0; i < board_size.height; i++)
            for (int j = 0; j < board_size.width; j++)
                objects.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));

        // Create the cube coordinates
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++) {
                cube_points_background.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));
                cube_points.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, (-(s.squareSize*2))));
            }
        }

        bool found_chessboard = findChessboardCorners(gray_image, board_size, frame_corners);
        if (found_chessboard)
        {
            solvePnP(objects, Mat(frame_corners), overall_intrinsic, dist_coeffs, rvecs, tvecs);
            projectPoints(cube_points_background, rvecs, tvecs, overall_intrinsic, dist_coeffs, frame_points_background);
            projectPoints(cube_points, rvecs, tvecs, overall_intrinsic, dist_coeffs, frame_points);

            // possible line colors
            vector<Scalar> colors = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };

            // indexes of start and end points of lines to draw
            vector<tuple<int, int>> draw_points = { make_tuple(0,2), make_tuple(2,8), make_tuple(6,0), make_tuple(8,6) };

            for (size_t i = 0; i < draw_points.size(); i++)
            {
                Scalar color = colors[rand() % colors.size()], color_background = colors[rand() % colors.size()], color_dimension = colors[rand() % colors.size()];
                line(frame_undistorted, frame_points[get<0>(draw_points[i])], frame_points_background[get<0>(draw_points[i])], color, 3);
                line(frame_undistorted, frame_points_background[get<0>(draw_points[i])], frame_points_background[get<1>(draw_points[i])], color_background, 3);
                line(frame_undistorted, frame_points[get<0>(draw_points[i])], frame_points[get<1>(draw_points[i])], color_dimension, 3);
            }
        }

        imshow("Webcam", frame);
        imshow("Undistorted", frame_undistorted);

        camera >> frame;

        if (waitKey(10) == 27)
        {
            destroyWindow("Webcam");
            destroyWindow("Undistorted");
            running = false;
        }
    }
    camera.release();

    return 0;
}

//if (false)
//{
//    if (s.inputType == 1)
//    {
//        image_amount = 0;
//        clock_t image_taken_time = clock();
//        VideoCapture camera = s.inputCapture;
//        if (!camera.isOpened()) {
//            cerr << "ERROR: Could not open camera" << endl;
//            return 1;
//        }
//        namedWindow("Webcam", WINDOW_AUTOSIZE);
//        while (image_amount < 5)
//        {
//            Mat gray_image;
//            vector<Point2f> image_corners;

//            camera >> image;
//            image_size = image.size();
//            imshow("Webcam", image);
//            cvtColor(image, gray_image, COLOR_BGR2GRAY);
//            //bool found_chessboard = false;
//            bool found_chessboard = findChessboardCorners(gray_image, board_size, image_corners);

//            if (found_chessboard)
//            {
//                cornerSubPix(gray_image, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
//                drawChessboardCorners(image, board_size, Mat(image_corners), found_chessboard);

//                if (waitKey(10) == 32)
//                {
//                    image_points.push_back(image_corners);
//                    object_points.push_back(objects);

//                    image_amount += 1;
//                    image_taken_time = clock();

//                    std::cout << "printing" << image_amount;
//                    namedWindow("Photo " + to_string(image_amount), WINDOW_AUTOSIZE);
//                    imshow("Photo " + to_string(image_amount), image);
//                }
//            }
//            else {
//                if (waitKey(10) == 27)
//                {
//                    break;
//                }
//            }
//        }

//        camera.release();
//    }
//    else
//    {
//        image = s.nextImage();
//        image_size = image.size();
//        image_amount = 0;
//        while (!image.empty() and image_amount < s.nrFrames)
//        {
//            Mat gray_image;
//            vector<Point2f> image_corners;
//            cvtColor(image, gray_image, COLOR_BGR2GRAY);
//            bool found_chessboard = findChessboardCorners(gray_image, board_size, image_corners);

//            if (found_chessboard)
//            {
//                cornerSubPix(gray_image, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
//                drawChessboardCorners(image, board_size, Mat(image_corners), found_chessboard);

//                image_points.push_back(image_corners);
//                object_points.push_back(objects);
//            }
//            namedWindow("Image display");
//            imshow("window", image);
//            //waitKey(0);

//            image_amount += 1;
//            image = s.nextImage();

//            if (waitKey(10) == 27)
//            {
//                break;
//            }
//        }
//    }
//}