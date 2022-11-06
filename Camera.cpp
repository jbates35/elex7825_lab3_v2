#include "stdafx.h"

#include "Camera.h"

CCamera::CCamera()
{
	// Initialize with a default camera image size 
	init(Size(1000, 600));
}

CCamera::~CCamera()
{
}

void CCamera::init (Size image_size, int cam_id)
{
	_worldview = false;

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
	_cam_id = cam_id;

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);

	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();
	_trans_factor = (Mat1f(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);

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

void CCamera::calculate_real_extrinsic()
{
	Mat _R_mat3, _R_matrix, _R_matrix_inv;
	_R_mat3 = (Mat1f(3, 1) << (float)rvec[0], (float)rvec[1], (float)rvec[2]);
	Rodrigues(_R_mat3, _R_matrix_inv);                   // converts Rotation Vector to Matrix

	_R_matrix = _R_matrix_inv.inv();

	Mat rotation = Mat((Mat1f(4, 4) <<
		_R_matrix.at<float>(0, 0), _R_matrix.at<float>(0, 1), _R_matrix.at<float>(0, 2), 0,
		_R_matrix.at<float>(1, 0), _R_matrix.at<float>(1, 1), _R_matrix.at<float>(1, 2), 0,
		_R_matrix.at<float>(2, 0), _R_matrix.at<float>(2, 1), _R_matrix.at<float>(2, 2), 0,
		0, 0, 0, 1
		));

	Mat extrinsic_mat = Mat((Mat1f(4, 4) <<
		1, 0, 0, tvec[0],
		0, 1, 0, tvec[1],
		0, 0, 1, tvec[2],
		0, 0, 0, 1
		));

	Mat focus_mat = Mat((Mat1f(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0
		));

	int roll = -90;
	int pitch = 0;
	int yaw = 0;

	//Calculate angles
	float sx = sin((float)roll * PI / 180);
	float cx = cos((float)roll * PI / 180);
	float sy = sin((float)pitch * PI / 180);
	float cy = cos((float)pitch * PI / 180);
	float sz = sin((float)yaw * PI / 180);
	float cz = cos((float)yaw * PI / 180);

	Mat T = (Mat1f(4, 4) <<
		cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, 0,
		sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, 0,
		-1 * sy, cy * sx, cy * cx, 0,
		0, 0, 0, 1);

	_trans_factor = _cam_real_intrinsic * focus_mat * extrinsic_mat.inv() * rotation.inv() * T;
	rotate = rotation.inv() * T;
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
	
	board->draw(cv::Size(960, 1440), im, 10, 1);
	imwrite("ChArUcoBoard.png", im);
}

void CCamera::calibrate_board()
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	float size_aruco_square = 0.035; // MEASURE THESE
	float size_aruco_mark = 0.0175; // MEASURE THESE

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	// Collect data from live video 
	while (inputVideo.grab()) {
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
		if (corner_ids.size() > 0) {
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0) {
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0) {
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}

		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0) {
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
	for (unsigned int frame = 0; frame < filteredImages.size(); frame++) {
		Mat imageCopy = filteredImages[frame].clone();

		if (calib_id[frame].size() > 0) {

			if (allCharucoCorners[frame].total() > 0) {
				aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);
			}
		}

		imshow("out", imageCopy);
		char key = (char)waitKey(0);
		if (key == 27) break;
	}
}

void CCamera::detect_aruco(Mat& im, Mat& im_cpy)
{
	if (inputVideo.grab()) {

		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> charucoIds;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(im_cpy);
		
		vector< int > markerIds;
		vector< vector< Point2f > > markerCorners, rejectedMarkers;

		// detect markers
		aruco::detectMarkers(im, dictionary, markerCorners, markerIds, detectorParams,
			rejectedMarkers);

		// refind strategy to detect more markers
		aruco::refineDetectedMarkers(im, board, markerCorners, markerIds, rejectedMarkers,
			_cam_real_intrinsic, _cam_real_dist_coeff);

		// interpolate charuco corners
		int interpolatedCorners = 0;
		if (markerIds.size() > 0)
			interpolatedCorners =
			aruco::interpolateCornersCharuco(markerCorners, markerIds, im, charucoboard,
				charucoCorners, charucoIds, _cam_real_intrinsic, _cam_real_dist_coeff);

		// estimate charuco board pose
		bool validPose = false;
		if (_cam_real_intrinsic.total() != 0) 
			validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec);
		/*
		if (charucoIds.size() > 0)
			cv::aruco::drawDetectedCornersCharuco(im_cpy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));

		if (charucoCorners.size() > 0)
			aruco::drawDetectedCornersCharuco(im_cpy, charucoCorners, charucoIds);
			*/
		if (validPose) {
			pose_seen = true;
			
			//Dump values into trackbars
			_cam_setting_x = tvec[0] * 1000;
			_cam_setting_y = tvec[1] * 1000;
			_cam_setting_z = tvec[2] * 1000;
		}

		/*
		if (pose_seen)
			//Draw frame axis on corner of grid
			cv::drawFrameAxes(im_cpy, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec, 0.5f * ((float)min(board_size.width, board_size.height) * (size_aruco_square)));
			*/
	
	}
}

void CCamera::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
	Mat trans_factor = _cam_virtual_intrinsic * _cam_virtual_extrinsic.inv();
	Mat pts = trans_factor * pt3d_mat;
	//Divide x and y by z
	pt = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), (pts.at<float>(1) / pts.at<float>(2)));
}

void CCamera::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	pts2d.clear();

	Mat trans_factor = _cam_virtual_intrinsic * _cam_virtual_extrinsic.inv();

	for (auto x : pts3d_mat) {
		//Crush 3d to 2d
		Mat pts = trans_factor * x;
		//Divide x and y by z
		Point2f pt2d = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), (pts.at<float>(1) / pts.at<float>(2)));
		pts2d.push_back(pt2d);
	}
}

void CCamera::transform_to_image_real(Mat pt3d_mat, Point2f& pt)
{
	Mat pts = _trans_factor * pt3d_mat;
	//Divide x and y by z
	pt = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), (pts.at<float>(1) / pts.at<float>(2)));
}

void CCamera::transform_to_image_real(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	pts2d.clear();

	for (auto x : pts3d_mat) {
		//Crush 3d to 2d
		Mat pts = (_trans_factor * x);
		//Divide x and y by z
		Point2f pt2d = cv::Point2f(pts.at<float>(0) / pts.at<float>(2), (pts.at<float>(1) / pts.at<float>(2)));
		pts2d.push_back(pt2d);
	}
}

//Project Points look up

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

	//cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();

	//calculate real world extrinsic
	if (_worldview)
		calculate_real_extrinsic();

	if (testing) {
		if ((cv::getTickCount() - refresh) / cv::getTickFrequency() >= REFRESH_INT*3) {
			refresh = cv::getTickCount();
			std::cout << "Intrinsic cam: " << std::endl << _cam_virtual_intrinsic << std::endl; // delme
			std::cout << "Extrinsic cam: " << std::endl << _cam_virtual_extrinsic << std::endl; //delme
		}
	}
}

void CCamera::set_lab(int lab)
{
	_lab = lab;
}

void CCamera::enable_worldview()
{
	_worldview = true;

	// Board settings
	board_size = Size(5, 7);
	dictionary_id = aruco::DICT_6X6_250;

	size_aruco_square = (float)MODEL_SCALE * 35 / 1000; // MEASURE THESE
	size_aruco_mark = (float)MODEL_SCALE * 35 / 2000; // MEASURE THESE

	detectorParams = aruco::DetectorParameters::create();
	dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_aruco_square, size_aruco_mark, dictionary);
	board = charucoboard.staticCast<aruco::Board>();

	inputVideo.open(_cam_id, CAP_DSHOW);

	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	filename = "C:\\Users\\jbate\\source\\repos\\jbates35\\elex7825_lab3_v2\\cam_param.xml";

	load_camparam(filename, _cam_real_intrinsic, _cam_real_dist_coeff);
	_cam_real_intrinsic.convertTo(_cam_real_intrinsic, CV_32FC1);

	pose_seen = false;

	if (testing)
		std::cout << "Camera matrix is... \n" << _cam_real_intrinsic << "\n\nDist Coefficients Matrix is...\n" << _cam_real_dist_coeff << "\n\n";
}
