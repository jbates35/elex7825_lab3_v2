#include "stdafx.h"

#include "Camera.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1000, 600));
}

CCamera::~CCamera()
{
}

void CCamera::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 0;

	_cam_setting_x = 0; // units in mm
	_cam_setting_y = 0; // units in mm
	_cam_setting_z = 500; // units in mm

	_cam_setting_roll = 0; // units in degrees
	_cam_setting_pitch = 0; // units in degrees
	_cam_setting_yaw = 0; // units in degrees

	_size = image_size;

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);

	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();

	//Tests
	refresh = cv::getTickCount();
	testing = false;
}

void CCamera::calculate_intrinsic()
{
	Mat mult1 = (Mat1f(3, 3) << 1 / _pixel_size, 0, _principal_point.x, 0, 1 / _pixel_size, _principal_point.y, 0, 0, 1);
	Mat mult2 = (Mat1f(3, 4) << (float) _cam_setting_f / 1000, 0, 0, 0, 0, (float) _cam_setting_f / 1000, 0, 0, 0, 0, 1, 0);

	_cam_virtual_intrinsic = mult1 * mult2;
}

void CCamera::calculate_extrinsic()
{
	//Calculate angles
	float sx = sin((float) _cam_setting_roll * PI /180);
	float cx = cos((float) _cam_setting_roll * PI / 180);
	float sy = sin((float) _cam_setting_pitch * PI / 180);
	float cy = cos((float) _cam_setting_pitch * PI / 180);
	float sz = sin((float) _cam_setting_yaw * PI / 180);
	float cz = cos((float) _cam_setting_yaw * PI / 180);

	_cam_virtual_extrinsic = (Mat1f(4, 4) << 
		cz*cy, cz*sy*sx-sz*cx, cz*sy*cx + sz*sx, (float) _cam_setting_x/1000,
		sz*cy, sz*sy*sx + cz*cx, sz*sy*cx-cz*sx, (float) _cam_setting_y / 1000,
		-1*sy, cy*sx, cy*cx, (float) _cam_setting_z / 1000,
		0, 0, 0, 1);
}

bool CCamera::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

bool CCamera::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);
	
	if (!fs.isOpened())
	{
		return false;
	}
	
	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

void CCamera::createChArUcoBoard()
{
	Mat im;
	float size_square = 0.06; // user specified
	float size_mark = 0.03; // user specified
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_id);
	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
	
	board->draw(cv::Size(600, 500), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCamera::calibrate_board(int cam_id)
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;
  
	float size_aruco_square = (float) 17/1000; // MEASURE THESE
	float size_aruco_mark = (float) 8.5/1000; // MEASURE THESE

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	VideoCapture inputVideo;
	inputVideo.open(cam_id, CAP_DSHOW);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	// Collect data from live video 
	while (inputVideo.grab()) 
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(draw_im);
		
		// First pass detect markers
		aruco::detectMarkers(im, dictionary, corners, corner_ids, detectorParams, rejected_corners);
		// Second pass detect markers
		aruco::refineDetectedMarkers(im, board, corners, corner_ids, rejected_corners);

		// Refine charuco corners
		if (corner_ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0)
		{
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0)
		{
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}
		
		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0) 
		{
			cout << "Frame captured" << endl;
			calib_corner.push_back(corners);
			calib_id.push_back(corner_ids);
			calib_im.push_back(im);
			calib_im_size = im.size();
		}
	}

	if (calib_id.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(calib_corner.size());
	for (unsigned int i = 0; i < calib_corner.size(); i++) {
		markerCounterPerFrame.push_back((int)calib_corner[i].size());
		for (unsigned int j = 0; j < calib_corner[i].size(); j++) {
			allCornersConcatenated.push_back(calib_corner[i][j]);
			allIdsConcatenated.push_back(calib_id[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, calib_im_size, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

	// prepare data for charuco calibration
	int nFrames = (int)calib_corner.size();
	vector< Mat > allCharucoCorners;
	vector< Mat > allCharucoIds;
	vector< Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for (int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		Mat currentCharucoCorners, currentCharucoIds;
		aruco::interpolateCornersCharuco(calib_corner[i], calib_id[i], calib_im[i], charucoboard,
			currentCharucoCorners, currentCharucoIds, cameraMatrix,
			distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(calib_im[i]);
	}

	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	// calibrate camera using charuco
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, calib_im_size, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = save_camparam("cam_param.xml", cameraMatrix, distCoeffs);
	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Rep Error Aruco: " << arucoRepErr << endl;
	cout << "Calibration saved to " << "cam_param.xml" << endl;

	// show interpolated charuco corners for debugging
		for (unsigned int frame = 0; frame < filteredImages.size(); frame++) 
		{
			Mat imageCopy = filteredImages[frame].clone();
			
			if (calib_id[frame].size() > 0) {

				if (allCharucoCorners[frame].total() > 0) 
				{
					aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],allCharucoIds[frame]);
				}
			}

			imshow("out", imageCopy);
			char key = (char)waitKey(0);
			if (key == 27) break;
	}
}

void CCamera::detect_aruco(Mat& im, Mat& im_cpy, int cam_id)
{
}

void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	Mat trans_factor = _cam_virtual_intrinsic * _cam_virtual_extrinsic;
	Mat pts = trans_factor * pt3d_mat;
	//Divide x and y by z
	pt = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), _size.height - (pts.at<float>(1) / pts.at<float>(2)));
}

void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	pts2d.clear();

	Mat trans_factor = _cam_virtual_intrinsic * _cam_virtual_extrinsic;

	for (auto x : pts3d_mat) {
		//Crush 3d to 2d
		Mat pts = trans_factor * x;
		//Divide x and y by z
		Point2f pt2d = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), _size.height - (pts.at<float>(1) / pts.at<float>(2)));
		pts2d.push_back(pt2d);
	}
}

void CCamera::update_settings(Mat &im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 350, "Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");
	
	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::checkbox(im, _camera_setting_window.x, _camera_setting_window.y, "Track Board", &track_board);

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();

	if (testing) {
		if ((cv::getTickCount() - refresh) / cv::getTickFrequency() >= REFRESH_INT*3) {
			refresh = cv::getTickCount();
			std::cout << "Intrinsic cam: " << std::endl << _cam_virtual_intrinsic << std::endl; // delme
			std::cout << "Extrinsic cam: " << std::endl << _cam_virtual_extrinsic << std::endl; //delme
		}
	}
}
