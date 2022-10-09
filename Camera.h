#pragma once

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace dnn;

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

public:
	void init(Size image_size);

	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	void createChArUcoBoard();
	void calibrate_board(int cam_id);

	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	void update_settings(Mat &im);
};

