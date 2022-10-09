////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created November 6, 2021 by Craig Hennessey
// Last updated Nov 6, 2021
////////////////////////////////////////////////////////////////
#pragma once

using namespace std;
using namespace cv;

class CuArm
{
public:
	CuArm();
	~CuArm();
private:
	Serial _com;

	Point3f _home_pos;
	Point3f _end_effector_pos;
	
	bool _gripper;
	bool _pump;
	float _speed;

public:
	void init_com (string comport);
	void init_robot ();

	bool send_com(string tx_str);
	bool get_com(string &rx_str);

	int set_motion_feedback(bool state);
	int set_pump(bool state);
	int set_gripper(bool state);

	bool set_pos(Point3f pos);
};

