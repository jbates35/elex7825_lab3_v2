#pragma once

#include <opencv2/opencv.hpp>

#include "Camera.h"
#include "uArm.h"

using namespace std;
using namespace cv;
using namespace dnn;

#define ANIMATE_INCREMENT 5
#define MAX_ANIMATE_RANGE 200

#define ARM_LENGTH 0.15 // Length of robot arm (a1, a2)
#define MAX_ICOORD 325
#define MIN_ICOORD 50.5

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
	vector<int> _joint_min, _joint_max;
	vector<string> _joint_names;
	vector<Point2f> rotate_robot(vector<Point3f>);
	int _stage, _count;


	//////////////////////////////////
	// LAB 6
	vector<string> _idir;
	vector<int> _icoord;
	vector<int> _icoord_min, _icoord_max;
	bool _worldview;
	vector<int> ikine_joints(int x_in = 0, int y_in = 0, bool positive = true);
	int _equation_select;
	int _angle;
	bool _kin_select; // false for froward kinematics, true for inverse kinematics
	Mat _current_view;
	void check_make_positive();
	Point2i _start_point;
	Point2i _increment;
	vector<Point2i> _corner_point;
	vector<Point2i> _corner_incs;
	Point2f _diff_norm;
	int _do_animate_inv;
	int _istage, _icount;
	int _istart;

  
public:
	Mat createHT(Vec3d t, Vec3d r);
	void set_worldview();

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

