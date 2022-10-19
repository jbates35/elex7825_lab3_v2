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
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;
	Mat _canvas_copy;

	struct box {
		vector<Mat> shape;
		Scalar color;
		//Point2f pos;
	};
	vector<box> _simple_robot;

	CCamera _virtualcam;

	VideoCapture input_video;
	Mat test_img;

	double test_timer;

	//CuArm uarm;


	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	vector<Point2f> rotate_robot(vector<Point3f>);

public:
	Mat createHT(Vec3d t, Vec3d r);

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

};

