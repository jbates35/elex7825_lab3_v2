#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>
//#include <opencv2/aruco/aruco_calib_pose.hpp>

#include "cvui.h"

CRobot::CRobot(int lab)
{
	//Set lab
	set_lab(lab);
	_worldview = false;

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

	//Boundaries of joint angles or distances (4th entry)
	_joint_min = { -180, -180, -25, -180 };
	_joint_max = { 180, 180, 175, 180 };

	//Boundaries of joint angles or distances (4th entry)
	_icoord_min = { -325, -325, 0, -180 };
	_icoord_max = { 325, 325, 200, 180 };

	_joint_names = { "J1", "J2", "P", "J3" };
	_idir = {"X","Y","Z","R"};
	
	_corner_point = {
		Point2i(MAX_ANIMATE_RANGE,MAX_ANIMATE_RANGE),
		Point2i(-1 * MAX_ANIMATE_RANGE,MAX_ANIMATE_RANGE),
		Point2i(-1 * MAX_ANIMATE_RANGE,-1 * MAX_ANIMATE_RANGE),
		Point2i(MAX_ANIMATE_RANGE,-1 * MAX_ANIMATE_RANGE),
		Point2i(MAX_ANIMATE_RANGE,MAX_ANIMATE_RANGE),
		Point2i(-1 * MAX_ANIMATE_RANGE,MAX_ANIMATE_RANGE),
		Point2i(-1 * MAX_ANIMATE_RANGE,-1 * MAX_ANIMATE_RANGE),
		Point2i(MAX_ANIMATE_RANGE,-1 * MAX_ANIMATE_RANGE)
	};
	_corner_incs = {
		Point2i(-5, 0),
		Point2i(0,-5),
		Point2i(5, 0),
		Point2i(0, 5),
		Point2i(-5, 0),
		Point2i(0,-5),
		Point2i(5, 0),
		Point2i(0, 5)
	};

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
	_joint[2] = 75;

	_icoord.clear();
	for (int i = 0; i < 4; i++) _icoord.push_back(0);

	_stage = 0;
	_count = 0;

	_istage = 0;
	_icount = 0;
	_equation_select = true;

	_kin_select = false;

	_istart = 0;
	_istage = 0;
	_icount = 0;

	_do_animate_inv = 0;
}

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 500, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 25;
	
	//Forward kinematic parameters
	for (int i = 0; i < _joint.size(); i++) {
			cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[i], _joint_min[i], _joint_max[i]);
			cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, _joint_names[i]);

			_setting_window.y += 45;
	}

	// Buttons for robot
	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "FAnimate"))	{
		init();
		_do_animate = 1;
		_stage = 0;
	}


	if (cvui::button(im, _setting_window.x+110, _setting_window.y, 100, 30, "reset")) {
		init();
	}

	_setting_window.y += 55;


	//Inverse kinematic parameters

	if (_lab >= 6) {
		for (int i = 0; i < _joint.size(); i++) {
			cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_icoord[i], _icoord_min[i], _icoord_max[i]);
			cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, _idir[i]);

			_setting_window.y += 45;
		}
	}

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "IAnimate")) {
		_kin_select = true;
		_do_animate_inv = 1;
		_istage = 0;
		_icount = 0;
	}

	string kin_type = "forward";
	Scalar kin_color = RED;

	if (_kin_select) {
		kin_type = "inverse";
		kin_color = GREEN;
	}

	if (cvui::button(im, _setting_window.x + 110, _setting_window.y, 100, 30, kin_type)) {
		_kin_select = !_kin_select;
		check_make_positive();
	}

	_setting_window.y += 35;

	circle(im, Point2i(_setting_window.x + 160, _setting_window.y + 8), 8, kin_color, -1);

	//Animate1
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

	//Animate2
	//STATE MACHINE
	if (_do_animate_inv != 0) {
		Point2i diff;
		float diff_mag;
		int x;
		int y;
		int i;
		int inext;

		switch (_do_animate_inv) {
		case 1:
			//Find out the closest point
			x = _icoord[0];
			y = _icoord[1];
			_start_point = Point2i(x, y);

			if (x >= 0) {
				if (y >= 0) _istart = 0;
				else _istart = 3;
			}
			else {
				if (y >= 0) _istart = 1;
				else _istart = 2;
			}

			//Calculate unit vector 
			diff = _corner_point[_istart] - _start_point;
			diff_mag = sqrt(diff.x * diff.x + diff.y * diff.y);
			_diff_norm = Point2f((float)diff.x / diff_mag, (float)diff.y / diff_mag);

			_icount = 1;
			_istage = 0;
			_do_animate_inv++;
			break;
		case 2:
			//Move towards the goal
			_icoord[0] = (int)round(_start_point.x + _icount * ANIMATE_INCREMENT * _diff_norm.x);
			_icoord[1] = (int)round(_start_point.y + _icount * ANIMATE_INCREMENT * _diff_norm.y);

			//Make them the same
			if (abs(_icoord[0] - _corner_point[_istart].x) < 5)
				_icoord[0] = _corner_point[_istart].x;			
			if (abs(_icoord[1] - _corner_point[_istart].y) < 5)
				_icoord[1] = _corner_point[_istart].y;

			_icount++;
			if (_icoord[0] == _corner_point[_istart].x && _icoord[1] == _corner_point[_istart].y) {
				_do_animate_inv++;
				_icount = 1;
			}
			break;
		case 3:
			i = _istart + _istage;
			inext = i + 1;

			//Move towards the goal
			_icoord[0] = _icoord[0] + _corner_incs[i].x;
			_icoord[1] = _icoord[1] + _corner_incs[i].y;

			//Make them the same
			if (abs(_icoord[0] - _corner_point[inext].x) <= 5)
				_icoord[0] = _corner_point[inext].x;			
			if (abs(_icoord[1] - _corner_point[inext].y) <= 5)
				_icoord[1] = _corner_point[inext].y;

			_icount++;

			if (abs(_icoord[0] - _corner_point[inext].x) < 5 && abs(_icoord[1] - _corner_point[inext].y) < 5) {
				_istage++;
				_icount = 1;
			}

			if (_istage == 4) {
				_do_animate_inv++;				
				
				//Get new point and increase stage
				diff = _start_point-_corner_point[_istage+_istart];
				diff_mag = sqrt(diff.x * diff.x + diff.y * diff.y);
				_diff_norm = Point2f((float)diff.x / diff_mag, (float)diff.y / diff_mag);
				_icount = 1;
			}
			break;
		case 4:
			//Move towards the goal
			_icoord[0] = (int)round(_corner_point[_istage + _istart].x + _icount * ANIMATE_INCREMENT * _diff_norm.x);
			_icoord[1] = (int)round(_corner_point[_istage + _istart].y + _icount * ANIMATE_INCREMENT * _diff_norm.y);

			//Make them the same
			if (abs(_icoord[0] - _start_point.x) < 5)
				_icoord[0] = _start_point.x;
			if (abs(_icoord[1] - _start_point.y) < 5)
				_icoord[1] = _start_point.y;

			_icount++;
			if (abs(_icoord[0] - _start_point.x) < 5 && abs(_icoord[1] - _start_point.y) < 5)
				init();
			break;
		default:
			_do_animate_inv = 0;

		} // end switch _do_animate_inv


	} // end if _do_animate_inv

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

void CRobot::set_worldview()
{
	_worldview = true;
	_virtualcam.enable_worldview();
}

void CRobot::set_lab(int lab)
{
	_lab = lab;
	_virtualcam.set_lab(lab);
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

	if (_worldview == false) {
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

	if (_worldview==false) {
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

void CRobot::create_lab5()
{
	_lab5_robot.clear();
	vector<Scalar> colors = { WHITE, RED, GREEN, BLUE };

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);
	_canvas_copy = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	int x_fix = 0;
	if (_worldview) x_fix = -90;


	// roll pitch yaw x y z
	vector<Mat> transpose_box = {
		extrinsic(x_fix, 0, 90, 0, 0, 0),
		extrinsic(0, 0, 90, 0.175, 0, 0, false),
		extrinsic(0, 0, 0, 0.15, 0, 0, false),
		extrinsic(0, 0, 90, 0.15, -1*(float)_joint[2] / 1000, 0, false)
	};

	vector<Mat> rotate_box = {
		extrinsic(),
		extrinsic(0,_joint[0]),
		extrinsic(0,_joint[1]),
		extrinsic(_joint[3])
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
	if(_worldview) 
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

/////////////////LAB 6
//Lab 6
vector<int> CRobot::ikine_joints(int x_in, int y_in, bool positive)
{
	float x = (float)x_in;
	float y = (float)y_in;

	float q1, q2;

	if (x == 0) x = 0.000001;
	if (y == 0) y = 0.000001;


	float num1 = sqrt(-66015625 + 106250 * y * y - y * y * y * y + 106250 * x * x - 2 * x * x * y * y - x * x * x * x);
	float denom1 = -8125 + y * y + 300 * x + x * x;

	float num2 = sqrt(-1 * (-625 + y * y + x * x) * (-105625 + y * y + x * x));
	float denom2 = -625 + y * y + x * x;
	
	if (positive) {
		q1 = round(360 / PI * atan2(300 * y + num1, denom1));
		q2 = round(-360 / PI * atan2(num2, denom2));
	}
	else {
		//Alternate equations
		q1 = round(360 / PI * atan2(300 * y - num1, denom1));
		q2 = round(360 / PI * atan2(num2, denom2));
	}

	if (q1 > 180) q1 -= 360;
	if (q1 < -180) q1 += 360;
	if (q2 > 180) q2 -= 360;
	if (q2 < -180) q2 += 360;

	vector<int> q = { (int)q1, (int)q2 };
	return q;
}


void CRobot::ikine()
{
		//Figure out the magnitude of the x and y
		float radius = sqrt(_icoord[0] * _icoord[0] + _icoord[1] * _icoord[1]);

		if (radius <= MAX_ICOORD && radius > MIN_ICOORD) {
			// a1, a2 from x, y
			vector<int> q;
			q = ikine_joints(_icoord[0], _icoord[1], _equation_select);
			_joint[0] = q[0];
			_joint[1] = q[1];
		}

		//Z
		_joint[2] = _icoord[2] - 25;

		//Theta
		_joint[3] = _icoord[3] - _joint[0] - _joint[1];
}

void CRobot::fkine()
{	
	float r11, r21, r31, r32, r33;
	r11 = _current_view.at<float>(0, 0);
	r21 = _current_view.at<float>(1, 0);
	r31 = _current_view.at<float>(2, 0);
	r32 = _current_view.at<float>(2, 1);
	r33 = _current_view.at<float>(2, 2);

	float pitch = atan2(-1 * r31, sqrt(r11 * r11 + r21 * r21));
	float roll = 180 / PI * atan2(r32 / cos(pitch), r33 / cos(pitch));
	float angle = (int)round(roll);
	if (angle > 180) angle -= 360;
	if (angle <= -180) angle += 360;

	float x, y;
	float q1 = PI / 180 * _joint[0];
	float q2 = PI / 180 * _joint[1];

	x = 175 * cos(q1 + q2) + 150 * cos(q1);
	y = 175 * sin(q1 + q2) + 150 * sin(q1);

	_icoord[0] = (int)round(x);
	_icoord[1] = (int)round(y);
	_icoord[2] = _joint[2] + 25;
	_icoord[3] = angle;

}

void CRobot::check_make_positive()
{	
	//Match current joints moving forward
	vector<int> q = ikine_joints(_icoord[0], _icoord[1], true);
	if (q[0] == _joint[0] && q[1] == _joint[1]) _equation_select = true;

	q.clear();
	q = ikine_joints(_icoord[0], _icoord[1], false);
	if (q[0] == _joint[0] && q[1] == _joint[1]) _equation_select = false;
}

void CRobot::draw_lab6()
{
	if (_worldview)
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

		if (_virtualcam.get_pose_seen() || _worldview==false) {
			//Draw box + worldview
			drawCoord(_canvas_copy, O, 5);
			drawBox(_canvas_copy, x.shape, x.color, 5);
		}
	}

	//Draw last worldview (end effector)
	Mat effector_translate = extrinsic(0, 0, 0, 0.15, 0, 0, false);

	current_view = current_view * effector_translate;// *derotate_robot.inv();

	//Store into member variable
	_current_view = current_view;

	if (_kin_select)
		ikine();
	else
		fkine();

	//FLIP
	current_view *= extrinsic(0, 0, 180);

	std::vector<Mat> O = createCoord();
	transformPoints(O, current_view);

	//Draw coordinates if pose is seen
	if (_virtualcam.get_pose_seen() || _worldview==false)
		drawCoord(_canvas_copy, O, 5);


	cv::imshow(CANVAS_NAME, _canvas_copy);

}

