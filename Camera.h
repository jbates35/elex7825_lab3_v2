#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <cmath>
#include "cvui.h"


using namespace std;
using namespace cv;
using namespace dnn;

#define REFRESH_INT 1

#define PI 3.14159265359

#define MODEL_SCALE 1

class CCamera
{
public:
	CCamera();
	~CCamera();

private:
	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	void calculate_intrinsic();
	void calculate_extrinsic();
	void calculate_real_extrinsic();

	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;

	// Real webcam
	Mat _cam_real_intrinsic;
	Mat _cam_real_extrinsic;
	Mat _cam_real_dist_coeff;
	Mat _trans_factor;

	//Time 
	double refresh, refresh_prev;

	Size _size;

	//For detecting charuco and creating such
	Size board_size;
	int dictionary_id;
	float size_aruco_square, size_aruco_mark;

	Ptr<aruco::DetectorParameters> detectorParams;
	Ptr<aruco::Dictionary> dictionary;

	Ptr<aruco::CharucoBoard> charucoboard;
	Ptr<aruco::Board> board;

	VideoCapture inputVideo;

	//Calibration
	std::string filename;
	cv::Mat cameraMatrix, distCoeffs;
	cv::Vec3d rvec, tvec;

	//Getting new point
	Point2f _board_pose_2d;
	Mat _new_pt_3d, _intrinsic_cam;

	int _lab;

	bool pose_seen; // false on initialize, true once pose has been seen

public:
	void init(Size image_size, int cam_id=0);

	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	void createChArUcoBoard();
	void calibrate_board();
	void detect_aruco(Mat& im, Mat& im_cpy);

	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	void transform_to_image_real(Mat pt3d_mat, Point2f& pt);
	void transform_to_image_real(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	void update_settings(Mat &im);

	bool get_pose_seen() { return pose_seen; }

	void set_lab(int lab);

	Mat rotate;

	bool testing;

	cv::Vec3d get_rvec() { return rvec; }
	cv::Vec3d get_tvec() { return tvec; }
};

