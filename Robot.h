#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;

#define WHITE Scalar(255, 255, 255)
#define RED Scalar(0, 0, 255)
#define GREEN Scalar(0, 255, 0)
#define BLUE Scalar(255, 0, 0)
#define YELLOW Scalar(0, 255, 255)
#define MAGENTA Scalar(255, 0, 255)
#define TEAL Scalar(255, 255, 0)

#define FILEPATH "C:\\Users\\jbate\\OneDrive\\Documents\\aruco_pics\\1.jpg"

class CRobot
{
public:
	CRobot(int lab = 0);
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;
	Mat _canvas_copy;
	void init();

	//Lab 3 version
	struct box {
		vector<Mat> shape;
		Scalar color;
		//Point2f pos;
	};
	vector<box> _simple_robot;

	struct box_l5 {
		vector<Mat> shape;
		Scalar color;
		Mat transpose;
		Mat rotate;
	};

	int _lab;
	void set_lab(int lab);

	vector<box_l5> _lab5_robot;


	CCamera _virtualcam;

	VideoCapture input_video;
	Mat test_img;

	double test_timer, turn_timer;

	//CuArm uarm;


	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour, int lab=3);
	void drawCoord(Mat& im, std::vector<Mat> coord3d, int lab = 3);
		
	////////////////////////////////////
	// LAB 5

	int _do_animate; // Animation state machine

	void update_settings(Mat& im);
	cv::Mat extrinsic(int roll = 0, int pitch = 0, int yaw = 0, float x = 0, float y = 0, float z = 0, bool normal = true);
	
	//Joint angles
	vector<int> _joint, _joint_disabled;
	Mat _world_view;
	vector<int> _joint_min, _joint_max;
	vector<Point2f> rotate_robot(vector<Point3f>);
	int _stage, _count;


	//////////////////////////////////
	// LAB 6
	vector<string> _idir;
	vector<int> _ijoint;
	int _istage, _icount;
	bool _worldview;
	vector<float> ikine_joints(bool positive = true);
  
public:
	Mat createHT(Vec3d t, Vec3d r);
	void set_worldview() { _worldview = true; }

	/////////////////////////////
	// Lab 3

	void create_simple_robot();
	void draw_simple_robot();

	/////////////////////////////
	// Lab 4
	void create_more_complex_robot();
	void draw_more_complex_robot();


	/////////////////////////////
	// Lab 5	
	void fkine(); // Input joint variables, output end effector pose
	void create_lab5();
	void draw_lab5(); 

	/////////////////////////
	// Lab 6
	void ikine();
	void draw_lab6();
};

