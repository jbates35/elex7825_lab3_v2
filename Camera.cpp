#include "stdafx.h"

#include "Camera.h"

CCamera::CCamera()
{

	testing = true;

	// Initialize with a default camera image size 
	init(Size(1000, 600));
}

CCamera::~CCamera()
{
}

Point3i CCamera::convert_to_angle(Mat rotate)
{
	float r11, r21, r31, r32, r33;
	r11 = rotate.at<float>(0, 0);
	r21 = rotate.at<float>(1, 0);
	r31 = rotate.at<float>(2, 0);
	r32 = rotate.at<float>(2, 1);
	r33 = rotate.at<float>(2, 2);

	float pitch_rads = atan2(-1 * r31, sqrt(r11 * r11 + r21 * r21));
	float roll_rads = atan2(r32 / cos(pitch_rads), r33 / cos(pitch_rads));
	float yaw_rads = atan2(r21 / cos(pitch_rads), r11 / cos(pitch_rads));

	return Point3i(
		floor(180 / PI * roll_rads),
		floor(180 / PI * pitch_rads),
		floor(180 / PI * yaw_rads)
	);
}


bool CCamera::markers_found()
{
	return box_poses.size()>=2;
}

int CCamera::marker_count()
{
	return box_poses.size();
}

vector<int> CCamera::get_pose(int curr)
{
	if (curr >= _marker_tvec.size())
		curr = 0;

	return box_poses[curr];
}


cv::Mat CCamera::extrinsic(int roll, int pitch, int yaw, float x, float y, float z, bool normal)
{	
	//Calculate angles
	float sx = sin((float)roll * PI / 180);
	float cx = cos((float)roll * PI / 180);
	float sy = sin((float)pitch * PI / 180);
	float cy = cos((float)pitch * PI / 180);
	float sz = sin((float)yaw * PI / 180);
	float cz = cos((float)yaw * PI / 180);

	Mat rotate = (Mat1f(4, 4) <<
		cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, 0,
		sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, 0,
		-1 * sy, cy * sx, cy * cx, 0,
		0, 0, 0, 1);

	Mat translate = (Mat1f(4, 4) <<
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1
		);

	if (normal) return rotate * translate;
	else return translate * rotate;
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

	box.x = 0;
	box.y = 0;
	box.z = 0;
	box.pitch = 0;
	box.roll = 0;
	box.yaw = 0;
	_can_detect = false;
	//For detectin aruco boxes
	_pose_detected = false;
	_marker_found = { false, false, false };
	_marker_id = { 50, 60, 70 };

	//Tests
	refresh = cv::getTickCount();
	update_angle = false;

	rvec_prime = Point3i(0, 0, 0);
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

	_trans_factor = _cam_real_intrinsic * focus_mat * extrinsic_mat.inv() * rotation.inv();

	Mat nu_extrinsic_mat = Mat((Mat1f(4, 4) <<
		_R_matrix_inv.at<float>(0, 0), _R_matrix_inv.at<float>(0, 1), _R_matrix_inv.at<float>(0, 2), tvec[0],
		_R_matrix_inv.at<float>(1, 0), _R_matrix_inv.at<float>(1, 1), _R_matrix_inv.at<float>(1, 2), tvec[1],
		_R_matrix_inv.at<float>(2, 0), _R_matrix_inv.at<float>(2, 1), _R_matrix_inv.at<float>(2, 2), tvec[2],
		0, 0, 0, 1
		));

	if (_can_detect) {	
		
		//To dump tvec and rvec with variables solved in
		vector<int> coord_dump;
		box_poses.clear();

		for (int i = 0; i < _marker_tvec.size(); i++) {
			
			coord_dump.clear();

			/// Want Marker with respect to Board
			Mat box_Rod;
			Mat box_R;

			box_Rod = (Mat1f(3, 1) << (float)_marker_rvec[i][0], (float)_marker_rvec[i][1], (float)_marker_rvec[i][2]);


			//Convert box rvec and tvec to a matrix
			Rodrigues(box_Rod, box_R);
			Point3i angles = convert_to_angle(box_R);
			Mat box_T = extrinsic(angles.x, angles.y, angles.z, _marker_tvec[i][0], _marker_tvec[i][1], _marker_tvec[i][2], false);

			//New total matrix solve
			Mat total_box_T = nu_extrinsic_mat.inv() * box_T;
			Mat total_box_R = (Mat1f(3, 3) <<
				total_box_T.at<float>(0, 0), total_box_T.at<float>(0, 1), total_box_T.at<float>(0, 2),
				total_box_T.at<float>(1, 0), total_box_T.at<float>(1, 1), total_box_T.at<float>(1, 2),
				total_box_T.at<float>(2, 0), total_box_T.at<float>(2, 1), total_box_T.at<float>(2, 2)
				);
			angles = convert_to_angle(total_box_R);

			//Dump box variables
			box.x = total_box_T.at<float>(0, 3) * 1000;
			box.y = total_box_T.at<float>(1, 3) * 1000;
			box.z = total_box_T.at<float>(2, 3) * 1000;
			box.roll = angles.x;
			box.pitch = angles.y;
			box.yaw = angles.z;

			coord_dump.push_back(box.roll);
			coord_dump.push_back(box.pitch);
			coord_dump.push_back(box.yaw);
			coord_dump.push_back(box.x);
			coord_dump.push_back(box.y);
			coord_dump.push_back(box.z);

			box_poses.push_back(coord_dump);
			}
		}
			
	_trans_factor *= T;
	 rvec_prime = convert_to_angle(rotation * extrinsic(90));
	_cam_setting_roll = rvec_prime.x;
	_cam_setting_pitch = rvec_prime.y;
	_cam_setting_yaw = rvec_prime.z;
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

	float size_aruco_square = 0.034; // MEASURE THESE
	float size_aruco_mark = 0.017; // MEASURE THESE

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
		
		if (charucoIds.size() > 0)
			if(testing) cv::aruco::drawDetectedCornersCharuco(im_cpy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));

		if (charucoCorners.size() > 0)
			if(testing) aruco::drawDetectedCornersCharuco(im_cpy, charucoCorners, charucoIds);
			
		if (validPose) {
			pose_seen = true;
			update_angle = true;
			
			//Dump values into trackbars
			_cam_setting_x = tvec[0] * 1000;
			_cam_setting_y = tvec[1] * 1000;
			_cam_setting_z = tvec[2] * 1000;

			if(testing) aruco::drawDetectedMarkers(im_cpy, markerCorners, markerIds);

			//Find markers we care about
			vector< vector< Point2f > > temp_corners;

			//Dump marker vecs
			for (int i=0; i<markerIds.size(); i++)
				for (int j = 0; j < 3; j++) 
					if (markerIds[i] == _marker_id[j]) {

						//Marker is found, make it true
						_marker_found[j] = true;

						temp_corners.push_back(markerCorners[i]);

					} else {

						//Marker is not found, make it false
						_marker_found[j] = false;
					}

			//Estimate the markers we care about
			_marker_tvec.clear();
			_marker_rvec.clear();

			//Estimate the markers
			cv::aruco::estimatePoseSingleMarkers(temp_corners, 0.02, _cam_real_intrinsic, _cam_real_dist_coeff, _marker_rvec, _marker_tvec);

			if (_marker_tvec.size() > 0) {
				_pose_detected = true;
				_can_detect = true;
				
			} else {
				_can_detect = false;
			}

		}

		if (pose_seen)
			//Draw frame axis on corner of grid
			cv::drawFrameAxes(im_cpy, _cam_real_intrinsic, _cam_real_dist_coeff, rvec, tvec, 0.5f * ((float)min(board_size.width, board_size.height) * (size_aruco_square)));
	
			//Draw text and what not
	} 
}

void CCamera::detect_box(Mat& im, Mat& im_cpy)
{

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
	if (_worldview && update_angle) {
		calculate_real_extrinsic();
		update_angle = false;
	}

	if (testing) {
		if ((cv::getTickCount() - refresh) / cv::getTickFrequency() >= 0.4) {
			refresh = cv::getTickCount();

			for (int i = 0; i < _marker_tvec.size(); i++) {
				//std::cout << "tvec : " < i << " : \t" << _marker_tvec[i] << endl << endl;
				//std::cout << "rvec : " << i << " : \t" << _marker_rvec[i] << endl;									
			}
		}
	}

	if (testing) {
		cv::putText(im, "R = ", Point(250, 100), 0, 0.8, Scalar(255, 255, 0));
		cv::putText(im, "P = ", Point(250, 120), 0, 0.8, Scalar(255, 255, 0));
		cv::putText(im, "Ya = ", Point(250, 140), 0, 0.8, Scalar(255, 255, 0));
		cv::putText(im, "X = ", Point(250, 160), 0, 0.8, Scalar(255, 255, 0));
		cv::putText(im, "Y = ", Point(250, 180), 0, 0.8, Scalar(255, 255, 0));
		cv::putText(im, "Z = ", Point(250, 200), 0, 0.8, Scalar(255, 255, 0));

		for (int i = 0; i < box_poses.size(); i++) {
			for (int j = 0; j < box_poses[i].size(); j++) {
				cv::putText(im, to_string(box_poses[i][j]), Point(350 + i * 100, 100 + j*20), 0, 0.8, Scalar(255, 255, 0));
			}
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

	//if (testing)
		//std::cout << "Camera matrix is... \n" << _cam_real_intrinsic << "\n\nDist Coefficients Matrix is...\n" << _cam_real_dist_coeff << "\n\n";
}
