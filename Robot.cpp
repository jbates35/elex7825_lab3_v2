#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1000, 600);

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

  ///////////////////////////////////////////////
	// uArm setup

	//uarm.init_com("COM4");
	//uarm.init_robot();

	for (int i = 0; i < 4; i++) _joint.push_back(0);

	init();

	_world_view = extrinsic();

}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;
}

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 200;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	for (int i = 0; i < _joint.size(); i++) {
		cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[i], -180, 180);
		cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "J" + to_string(i));

		_setting_window.y += 45;
	}

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	if (_do_animate != 0)
	{
		int step_size = 5;
		if (_do_animate == 1)
		{
			// state 1
			if (1) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2
			if (1) { _do_animate = 3; }
		}
		else if (_do_animate == 3) {
			if (1) { _do_animate = 0; init(); }
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

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	_virtualcam.transform_to_image(box3d, box2d);


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

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d)
{
	Point2f O, X, Y, Z;

	_virtualcam.transform_to_image(coord3d.at(0), O);
	_virtualcam.transform_to_image(coord3d.at(1), X);
	_virtualcam.transform_to_image(coord3d.at(2), Y);
	_virtualcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::detect_charuco(Mat& im, Mat& im_copy)
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
	cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

	im.copyTo(im_copy);

	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f> > markerCorners;

	//cv::aruco::detectMarkers(image, board->getDictionary(), markerCorners, markerIds, params);

	//or
	cv::aruco::detectMarkers(im, dictionary, markerCorners, markerIds, params);

	// if at least one marker detected
	if (markerIds.size() > 0) {
		cv::aruco::drawDetectedMarkers(im_copy, markerCorners, markerIds);
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> charucoIds;
		cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, im, board, charucoCorners, charucoIds);
		// if at least one charuco corner detected
		if (charucoIds.size() > 0)
			cv::aruco::drawDetectedCornersCharuco(im_copy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
	}

}

void CRobot::create_simple_robot()
{
	vector<Point2f> translate = { Point2f(0.0, 0.0), Point2f(0.0, 0.05), Point2f(0.05, 0.1), Point2f(-0.05, 0.1), Point2f(0.0, 0.15) };
	vector<Scalar> colors = { RED, RED, GREEN, BLUE, RED };

	Mat transform = (Mat1f(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);

	for (int i = 0; i < translate.size(); i++) {

		box _box;
		_box.shape = createBox(0.05, 0.05, 0.05);
		_box.color = colors[i];

		transform.at<float>(0, 3) = translate[i].x;
		transform.at<float>(1, 3) = translate[i].y + 0.025;
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

void CRobot::create_more_complex_robot()
{
	_canvas = imread(FILEPATH, IMREAD_COLOR);
}

void CRobot::draw_more_complex_robot()
{
	_canvas = imread(FILEPATH, IMREAD_COLOR);
	detect_charuco(_canvas, _canvas_copy);

	cv::imshow(CANVAS_NAME, _canvas);
	cv::imshow("Copy of canvas", _canvas_copy);
}


void CRobot::fkine()
{
}

void CRobot::create_lab5()
{
	_lab5_robot.clear();

	box_l5 _box; // delme
	vector<Scalar> colors = { WHITE, RED, GREEN, BLUE };

	// roll pitch yaw x y z
	vector<Mat> transpose_box = {
		extrinsic(0, 0, 90, 0.075, 0, 0),
		extrinsic(0, 0, -90, 0.1, -0.075, 0, false),
		extrinsic(0, 0, 0, 0.15, 0, 0, false),
		extrinsic(0, 0, -90, 0.075, 0, 0, false)
	};

	vector<Mat> rotate_box_1 = {
		extrinsic(),
		extrinsic(_joint[0]),
		extrinsic(),
		extrinsic()
	};

	vector<Mat> rotate_box_2 = {
		extrinsic(),
		extrinsic(_joint[1]),
		extrinsic(),
		extrinsic()
	};

	Mat current_view = extrinsic();

	for (int i = 0; i < colors.size(); i++) {

		box_l5 _box;
		_box.shape = createBox(0.15, 0.05, 0.05);
		_box.color = colors[i];
		_box.transpose = transpose_box[i];
		_box.rotate_pre = rotate_box_1[i];
		_box.rotate_post = rotate_box_2[i];

		_lab5_robot.push_back(_box);
	}	
}


void CRobot::draw_lab5()
{
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	Mat current_view = extrinsic();

	for (auto x : _lab5_robot) {

		current_view = current_view* x.rotate_pre* x.transpose;
		transformPoints(x.shape, current_view);

		
		//Draw worldview
		


		drawBox(_canvas, x.shape, x.color);

		current_view *= x.rotate_post;
	}

	cv::imshow(CANVAS_NAME, _canvas);
}
