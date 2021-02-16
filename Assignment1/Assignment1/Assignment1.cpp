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
    const String keys
        = "{help h usage ? |           | print this message            }"
        "{@settings        |default.xml| input setting file            }"
        "{d                |           | actual distance between top-left and top-right corners of "
        "the calibration grid }"
        "{winSize          | 11        | Half of search window for cornerSubPix }";

    CommandLineParser parser(argc, argv, keys);
    Settings s;
    const string inputSettingsFile = parser.get<string>(0);
    FileStorage fs(inputSettingsFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        parser.printMessage();
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();

    Mat image;
    int image_amount;
    Size image_size;

    Size board_size = Size(s.boardSize);
    vector<vector<Point2f>> image_points;
    vector<vector<Point3f>> object_points;
    vector<Point3f> objects;

    for (int i = 0; i < board_size.height; i++)
        for (int j = 0; j < board_size.width; j++)
            objects.push_back(Point3f((float)j * s.squareSize, (float)i * s.squareSize, 0));

    if (s.inputType == 1)
    {
        image_amount = 0;
        clock_t image_taken_time = clock();
        VideoCapture camera = s.inputCapture;
        if (!camera.isOpened()) {
            cerr << "ERROR: Could not open camera" << endl;
            return 1;
        }

        namedWindow("Webcam", WINDOW_AUTOSIZE);
        while (image_amount < 5)
        {
            Mat gray_image;
            vector<Point2f> image_corners;

            camera >> image;
            image_size = image.size();
            imshow("Webcam", image);
            cvtColor(image, gray_image, COLOR_BGR2GRAY);
            //bool found_chessboard = false;
            bool found_chessboard = findChessboardCorners(gray_image, board_size, image_corners);

            if (found_chessboard)
            {
                cornerSubPix(gray_image, image_corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
                drawChessboardCorners(image, board_size, Mat(image_corners), found_chessboard);

                if (waitKey(10) == 32)
                {
                    image_points.push_back(image_corners);
                    object_points.push_back(objects);

                    image_amount += 1;
                    image_taken_time = clock();

                    std::cout << "printing" << image_amount;
                    namedWindow("Photo " + to_string(image_amount), WINDOW_AUTOSIZE);
                    imshow("Photo " + to_string(image_amount), image);
                }
            }
            else {
                if (waitKey(10) == 27)
                {
                    break;
                }
            }
        }

        camera.release();
    }
    else
    {
        image = s.nextImage();
        image_size = image.size();
        image_amount = 0;
        while (!image.empty() and image_amount < s.nrFrames)
        {
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
            //namedWindow("Image display");
            //imshow("window", image);
            //waitKey(0);

            image_amount += 1;
            image = s.nextImage();

            if (waitKey(10) == 27)
            {
                break;
            }
        }
    }

    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    calibrateCamera(object_points, image_points, image_size, intrinsic, distCoeffs, rvecs, tvecs);

    cout << "\n\n intrinsic:-\n" << intrinsic;
    cout << "\n\n distCoeffs:-\n" << distCoeffs;

    return 0;
}