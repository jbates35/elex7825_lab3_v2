#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>
//#include <opencv2/aruco/aruco_calib_pose.hpp>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);
	
	test_timer = cv::getTickCount();

  ///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();

	init();

	_world_view = extrinsic();

	//Boundaries of joint angles or distances (4th entry)
	_joint_min = { -180, -180, -180, -25 };
	_joint_max = { 180, 180, 180, 175 };

}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;

	_joint.clear();
	for (int i = 0; i < 4; i++) _joint.push_back(0);
	_joint[3] = 75;

	_stage = 0;
	_count = 0;

}

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 25;
	
	for (int i = 0; i < _joint.size(); i++) {
			cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[i], _joint_min[i], _joint_max[i]);
			cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J" + to_string(i));

			_setting_window.y += 45;
	}

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))	{
		init();
		_do_animate = 1;
		_stage = 0;
	}

	if (cvui::button(im, _setting_window.x+110, _setting_window.y, 100, 30, "reset")) {
		init();
	}

	//Animate
	if (_do_animate != 0) {
		int step_size = 10;
		int i = _do_animate - 1;

		switch (_stage) {
		case 0: // cw to -180
			_joint[i] -= step_size;
			if (_joint[i] <= _joint_min[i]) _stage = 1;
			break;
		case 1: // ccw to 180
			_joint[i] += step_size;
			if (_joint[i] >= _joint_max[i]) _stage = 2;
			break;
		case 2: // cw to 0
			_joint[i] -= step_size;
			if (_joint[i] <= (_joint_max[i] + _joint_min[i]) / 2) _stage = 3;
			break;
		case 3: // move on
			_joint[i] = (_joint_max[i] + _joint_min[i]) / 2;
			_stage = 0;
			_do_animate++;
			break;
		default:
			init();
		}

		if (_do_animate == 5) {
			_do_animate = 0;
			init();
		}
	}
	cvui::update();
}

cv::Mat CRobot::extrinsic(int roll, int pitch, int yaw, float x, float y, float z, bool normal)
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

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r)
{
	return (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;

	float axis_length = 0.05;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}

void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour, int lab)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	if (lab == 3) {
		_virtualcam.transform_to_image(box3d, box2d);
	} else {
		_virtualcam.transform_to_image_real(box3d, box2d);
	}

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);
		line(im, pt1, pt2, colour, 1);
	}

	if (_virtualcam.testing) {
		std::cout << box2d;
		for (auto x : box2d) {
			circle(_canvas, x, 1, RED, 2);
		}
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d, int lab)
{
	Point2f O, X, Y, Z;

	if (lab == 3) {
		_virtualcam.transform_to_image(coord3d.at(0), O);
		_virtualcam.transform_to_image(coord3d.at(1), X);
		_virtualcam.transform_to_image(coord3d.at(2), Y);
		_virtualcam.transform_to_image(coord3d.at(3), Z);
	} else {
		_virtualcam.transform_to_image_real(coord3d.at(0), O);
		_virtualcam.transform_to_image_real(coord3d.at(1), X);
		_virtualcam.transform_to_image_real(coord3d.at(2), Y);
		_virtualcam.transform_to_image_real(coord3d.at(3), Z);
	}

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

vector<Point2f> CRobot::rotate_robot(vector<Point3f>)
{
	return vector<Point2f>();
}

void CRobot::create_simple_robot()
{
	vector<Point2f> translate = { Point2f(0.0, 0.0), Point2f(0.0, 0.05/2), Point2f(0.05/2, 0.1/2), Point2f(-0.05/2, 0.1/2), Point2f(0.0, 0.15/2) };
	vector<Scalar> colors = { RED, RED, GREEN, BLUE, RED };

	Mat transform = (Mat1f(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);

	for (int i = 0; i < translate.size(); i++) {

		box _box;
		_box.shape = createBox(0.05/2, 0.05/2, 0.05/2);
		_box.color = colors[i];
		//_box.pos = ...

		transform.at<float>(0, 3) = translate[i].x;
		transform.at<float>(1, 3) = translate[i].y + 0.025/2;
		transformPoints(_box.shape, transform);
		_simple_robot.push_back(_box);
	}
}

void CRobot::draw_simple_robot()
{

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	_virtualcam.update_settings(_canvas);

	std::vector<Mat> O = createCoord();
	drawCoord(_canvas, O);

	for (auto x : _simple_robot) {
		drawBox(_canvas, x.shape, x.color);
	}

	cv::imshow(CANVAS_NAME, _canvas);
}

void CRobot::draw_more_complex_robot()
{
	_virtualcam.detect_aruco(_canvas, _canvas_copy);
	_virtualcam.update_settings(_canvas_copy);

	//Move robot
	Vec3d tvec = _virtualcam.get_tvec();

	for (auto x : _simple_robot) {
		if (_virtualcam.get_pose_seen())
			drawBox(_canvas_copy, x.shape, x.color, 4);
	}

	if ((getTickCount() - turn_timer) / getTickFrequency() >= 0.01) {
		turn_timer = getTickCount();

		int roll = 0;
		int pitch = 4;
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

		for (auto x : _simple_robot)
			transformPoints(x.shape, T);
	}
	//cv::imshow("7825 Canvas (test1)", _canvas);
	cv::imshow("7825 Canvas Lab 4", _canvas_copy);

}

void CRobot::fkine()
{
}

void CRobot::create_lab5()
{
	_lab5_robot.clear();
	vector<Scalar> colors = { WHITE, RED, GREEN, BLUE };

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	_canvas_copy = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// roll pitch yaw x y z
	vector<Mat> transpose_box = {
		extrinsic(0, 0, 90, 0, 0, 0),
		extrinsic(0, 0, 90, 0.175, 0, 0, false),
		extrinsic(0, 0, 0, 0.15, 0, 0, false),
		extrinsic(0, 0, 90, 0.15, -1*(float)_joint[3] / 1000, 0, false)
	};

	vector<Mat> rotate_box = {
		extrinsic(),
		extrinsic(0,_joint[0]),
		extrinsic(0,_joint[1]),
		extrinsic(_joint[2])
	};

	Mat current_view = extrinsic();

	for (int i = 0; i < colors.size(); i++) {

		box_l5 _box;
		_box.shape = createBox(0.15, 0.05, 0.05);
		_box.color = colors[i];
		_box.transpose = transpose_box[i];
		_box.rotate = rotate_box[i];

		transformPoints(_box.shape, extrinsic(0, 0, 0, 0.075));

		_lab5_robot.push_back(_box);
	}	
}


void CRobot::draw_lab5()
{

	_virtualcam.detect_aruco(_canvas, _canvas_copy);
	_virtualcam.update_settings(_canvas_copy);

	update_settings(_canvas_copy);

	Mat current_view = extrinsic();

	for (auto x : _lab5_robot) {

		//Change origin
		current_view = current_view * x.transpose * x.rotate;

		//Transform box
		transformPoints(x.shape, current_view);
		
		//Create worldview
		std::vector<Mat> O = createCoord();
		transformPoints(O, current_view);

		if (_virtualcam.get_pose_seen()) {
			//Draw box + worldview
			drawCoord(_canvas_copy, O, 5);
			drawBox(_canvas_copy, x.shape, x.color, 5);
		}
	}

	//Draw last worldview (end effector)
	Mat effector_translate = extrinsic(0, 0, 0, 0.15, 0, 0, false);

	//TODO **************************************************************
	Mat derotate_robot = extrinsic(0, 0, 90) * extrinsic(0, 0, 90) * _lab5_robot[1].rotate * _lab5_robot[2].rotate * extrinsic(0, 0, 90) * _lab5_robot[3].rotate;

	current_view = current_view * effector_translate * derotate_robot.inv();
	
	//FLIP
	current_view *= extrinsic(0, 0, 180);

	std::vector<Mat> O = createCoord();
	transformPoints(O, current_view);

	//Draw coordinates if pose is seen
	if (_virtualcam.get_pose_seen())
		drawCoord(_canvas_copy, O, 5);

	cv::imshow(CANVAS_NAME, _canvas_copy);
}

