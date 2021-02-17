// opencv-1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "Settings.cpp"
//#include "calibration.cpp"
//#include <parser.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
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

    Mat image;
    int image_amount;
    Size image_size;

    Size board_size = Size(s.boardSize);
    Mat overall_dist_coeffs;
    Mat overall_intrinsic = Mat(3, 3, CV_32FC1);

    overall_intrinsic.ptr<float>(0)[0] = 1;
    overall_intrinsic.ptr<float>(1)[1] = 1;

    bool done = false;
    double previous_rms = 10;
    while(done == false)
    {
        vector<vector<Point2f>> image_points;
        vector<vector<Point3f>> object_points;
        vector<Point3f> objects;

        for (int i = 0; i < board_size.height; i++)
            for (int j = 0; j < board_size.width; j++)
                objects.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));

        vector<String> images = s.imageList;
        for (int image_amount = 0; image_amount < images.size(); image_amount++)
        {
            image = imread(images[image_amount], IMREAD_COLOR);
            image_size = image.size();
            Mat gray_image;
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
            namedWindow("Image display");
            imshow("window", image);
            //waitKey(0);
            image = s.nextImage();

            if (waitKey(10) == 27)
            {
                break;
            }
        }

        Mat intrinsic = overall_intrinsic;
        Mat dist_coeffs = overall_dist_coeffs;
        vector<Mat> rvecs;
        vector<Mat> tvecs;

        double rms = calibrateCamera(object_points, image_points, image_size, intrinsic, dist_coeffs, rvecs, tvecs);
        if (rms < previous_rms)
        {
            overall_intrinsic = intrinsic;
            overall_dist_coeffs = dist_coeffs;
            previous_rms = rms;
        }
        else 
        {
            done = true;
        }

        std::cout << "\n Root mean square: " << rms;
        std::cout << "\n intrinsic: " << intrinsic;
        std::cout << "\n overall_intrinsic: " << overall_intrinsic;

        std::cout << "\n distCoeffs: " << dist_coeffs << "\n";
        std::cout << "\n overall_dist_coeffs: " << overall_dist_coeffs << "\n";
    }

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