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

namespace
{
	const char* about =
		"Calibration using a ChArUco board\n"
		"  To capture a frame for calibration, press 'c',\n"
		"  If input comes from video, press any key for next frame\n"
		"  To finish capturing, press 'ESC' key and calibration starts.\n";
	const char* keys =
		"{w        |       | Number of squares in X direction }"
		"{h        |       | Number of squares in Y direction }"
		"{sl       |       | Square side length (in meters) }"
		"{ml       |       | Marker side length (in meters) }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{cd       |       | Input file with custom dictionary }"
		"{@outfile |<none> | Output file with calibrated camera parameters }"
		"{v        |       | Input from video file, if ommited, input comes from camera }"
		"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
		"{dp       |       | File of marker detector parameters }"
		"{rs       | false | Apply refind strategy }"
		"{zt       | false | Assume zero tangential distortion }"
		"{a        |       | Fix aspect ratio (fx/fy) to this value }"
		"{pc       | false | Fix the principal point at the center }"
		"{sc       | false | Show detected chessboard corners after calibration }";
}

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

