#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include "calibration_settings.cpp"
#include <thread>
#include <filesystem>
#include <opencv2/features2d.hpp>
#include "camera_calibration.h"
#include <opencv2/highgui/highgui_c.h>

using namespace std;

namespace nl_uu_science_gmt
{
	camera_calibration::camera_calibration() { };

	void camera_calibration::calibration(int argc, char* argv[], string video_path, int camera_amount)
	{
		// Go through all of the videos
		for (size_t i = 1; i <= camera_amount; i++)
			calibrate(argc, argv, (video_path + "cam" + to_string(i) + "/"));
	};

	int camera_calibration::calibrate(int argc, char* argv[], string video_path)
	{
		// Read the settings from default.xml
		vector<String> imgs;
		calibration_settings s;
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
		Mat overall_dist_coeffs, overall_intrinsic = Mat(3, 3, CV_32F);
		vector<Mat> overall_tvecs, overall_rvecs;

		overall_intrinsic.ptr<float>(0)[0] = 1;
		overall_intrinsic.ptr<float>(1)[1] = 1;
		vector<float> overall_errors;
		bool done = false;
		float best_rms = 10;
		int loop = 0;

		vector<String> images;

		// Load the in intrinsics.avi of the current video
		VideoCapture vcamera;
		vcamera.open(video_path + "intrinsics.avi");

		if (!vcamera.isOpened())
		{
			cout << "Error opening file" << endl;
			return -1;
		}

		Mat prev_frame, prev_frame_gray, new_frame, new_frame_gray, diff_frame, video;
		int count = 0;
		int frame_num = 0;
		time_t current_time;

		string path = "/";
		while (true)
		{
			current_time = time(NULL);
			if (count == 0)
			{
				vcamera >> prev_frame;
			}
			vcamera >> video;
			vcamera >> new_frame;

			if (waitKey(10) == 27 || new_frame.empty())
			{
				break;
			}

			cvtColor(prev_frame, prev_frame_gray, COLOR_BGR2GRAY);
			cvtColor(new_frame, new_frame_gray, COLOR_BGR2GRAY);
			// Take difference of the previously saved frame vs the new frame
			absdiff(prev_frame_gray, new_frame_gray, diff_frame);

			// We keep the first frame and the new frame if it does exceed the threshold
			if (count == 0 || countNonZero(diff_frame) > 250000)
			{
				string img_name = video_path + "output/IMG_" + to_string(count) + ".png";
				imwrite(img_name, new_frame);

				images.push_back(img_name);

				prev_frame = new_frame.clone();
				count++;
			}
		}

		// While loop: iterates over all images leaving out image (1 + iteration number)
		while (done == false && loop < images.size()) {
			//std::cout << "Image set:" << loop << "\n";

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
			//cout << "Image set size:" << subset_images.size() << "\n";

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
			vector<Mat> rvecs = overall_rvecs, tvecs = overall_tvecs;

			// Run calibration with the parameters found in the current set and update high scores accordingly.
			float rms = calibrateCamera(object_points, image_points, image_size, intrinsic, dist_coeffs, rvecs, tvecs);

			// When the altered set has a significant better rms we replace the current best, and save the variables of this set
			if (abs(best_rms - rms) > 0.01) {
				overall_intrinsic = intrinsic;
				overall_dist_coeffs = dist_coeffs;
				overall_rvecs = rvecs;
				overall_tvecs = tvecs;
				best_rms = rms;
				images = subset_images;
			}
			loop++;
		}

		cout << "Calibration completed " << video_path << ".\n";
		cout << "Best rms: " << best_rms << "\n";
		cout << "Final intrinsic: " << overall_intrinsic << "\n";
		cout << "Final dist coeffs: " << overall_dist_coeffs << "\n";

		// Save the obtained intrinsic matrix in xml //
		FileStorage fsn(video_path + "intrinsics.xml", FileStorage::WRITE);
		fsn << "CameraMatrix" << overall_intrinsic;
		fsn << "DistortionCoeffs" << overall_dist_coeffs;
		fsn.release();

		return 0;
	};

	Mat contour_calc(InputArray image)
	{
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		Mat drawing = Mat::zeros(image.size(), CV_8UC3);
		for (size_t i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(0,0,255);
			drawContours(drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
		}

		return drawing;
	}

	void camera_calibration::color_model(string video_file, int camera_amount)
	{
		int max = 255;
		Mat image, image_background, hsv_image, hsv_background, tmp, foreground, background, image_final;

		image_background = imread(video_file + "cam1/background.png");
		image = imread(video_file + "cam1/4pers.png");
		
		while (!image.empty())
		{
			if (waitKey(10) == 27 || image.empty())
			{
				break;
			}

			cvtColor(image, hsv_image, CV_BGR2HSV);
			cvtColor(image_background, hsv_background, CV_BGR2HSV);

			vector<Mat> image_channels, camera_channels;
			// Split the HSV-channels for further analysis
			split(hsv_image, image_channels);
			// Split the HSV-channels for further analysis
			split(hsv_background, camera_channels);

			Scalar mean, m_h_stddev, m_s_stddev, m_v_stddev;
			// Compute channel 0 difference
			absdiff(image_channels.at(0), camera_channels.at(0), tmp);
			// Compute standard deviation of camera channel 1, to find idial h_threshold
			meanStdDev(tmp, mean, m_h_stddev);

			// Apply new threshold
			threshold(tmp, foreground, m_h_stddev[0], max, CV_THRESH_BINARY);
			// Background subtraction S
			absdiff(image_channels.at(1), camera_channels.at(1), tmp);

			// Compute standard deviation of camera channel 1, to find idial s_threshold
			meanStdDev(tmp, mean, m_s_stddev);

			// Apply new threshold
			threshold(tmp, background, m_s_stddev[0], max, CV_THRESH_BINARY);
			bitwise_and(foreground, background, foreground);

			// Background subtraction V
			absdiff(image_channels.at(2), camera_channels.at(2), tmp);
			// Compute standard deviation of camera channel 1, to find idial v_threshold
			meanStdDev(tmp, mean, m_v_stddev);

			// Apply new threshold
			threshold(tmp, background, (m_v_stddev[0] * 2), max, CV_THRESH_BINARY);
			bitwise_or(foreground, background, foreground);

			// Detecting noice
			Mat structured_elements_2 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
			Mat structured_elements_5 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

			// Remove small detected noice
			erode(foreground, foreground, structured_elements_2);
			dilate(foreground, foreground, structured_elements_2);

			// Remove large detected noice
			dilate(foreground, foreground, structured_elements_5);
			erode(foreground, foreground, structured_elements_5);

			copyTo(image, image_final, foreground);

			/*for i in range(length) : # find the biggest contour(according to area)
				temp = contours[i]
				area = cv2.contourArea(temp)
				if area > maxArea:
			maxArea = area
				ci = i*/

				


			// convert to float & reshape to a [3 x W*H] Mat 
//  (so every pixel is on a row of it's own)
			//Mat data, thres_data;
			//image_final.convertTo(data, CV_32F);
			//data = data.reshape(1, data.total());

			//Scalar shape_mean, shape_stddev;
			//meanStdDev(data, mean, shape_stddev);

			//threshold(data, thres_data, shape_stddev[0], max, CV_THRESH_BINARY);
			//bitwise_and(data, thres_data, data);

			//// do kmeans
			//Mat labels, centers;
			//kmeans(data, 8, labels, TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3,
			//	KMEANS_PP_CENTERS, centers);

			//// reshape both to a single row of Vec3f pixels:
			//centers = centers.reshape(3, centers.rows);
			//data = data.reshape(3, data.rows);

			//// replace pixel values with their center value:
			//Vec3f* p = data.ptr<Vec3f>();
			//for (size_t i = 0; i < data.rows; i++) {
			//	int center_id = labels.at<int>(i);
			//	p[i] = centers.at<Vec3f>(center_id);
			//}

			//// back to 2d, and uchar:
			//image_final = data.reshape(3, image.rows);
			//image_final.convertTo(image, CV_8U);

			///////////////////////////////////////////////////////////////////////////////////

			//vector<Mat> bgr_planes;

			//split(image, bgr_planes);
			//int histSize = 256;
			//float range[] = { 0, 256 }; //the upper boundary is exclusive
			//const float* histRange = { range };
			//bool uniform = true, accumulate = false;
			//Mat b_hist, g_hist, r_hist;
			//Mat hist;

			//calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
			//calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
			//calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

			//int hist_w = 512, hist_h = 400;
			//int bin_w = cvRound((double)hist_w / histSize);
			//Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
			//normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
			//normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
			//normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
			//for (int i = 1; i < histSize; i++)
			//{
			//	line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			//		Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
			//		Scalar(255, 0, 0), 2, 8, 0);
			//	line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
			//		Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
			//		Scalar(0, 255, 0), 2, 8, 0);
			//	line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
			//		Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
			//		Scalar(0, 0, 255), 2, 8, 0);
			//}

			imshow("Final image", image_final);
			imshow("Mask", foreground);
		}
	};
}