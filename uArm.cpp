////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created November 6, 2021 by Craig Hennessey
// Last updated Nov 6, 2021
////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "uArm.h"

CuArm::CuArm()
{
	// Robot Parameters
	_speed = 6000;
  _home_pos = Point3f(150, 0, 0);
	_end_effector_pos = _home_pos;
	_gripper = false;
	_pump = false;
}

CuArm::~CuArm()
{
}

bool CuArm::send_com(string tx_str)
{
	_com.write(tx_str.c_str(), tx_str.length());
	Sleep(10);

	return TRUE;
}

bool CuArm::get_com(string &rx_str)
{
	// temporary storage
	char buff[2];
	rx_str = "";

	// start timeout count
	float start_time = GetTickCount();
	buff[0] = 0;

	// If 1 byte was read then print to screen, timeout after 2 seconds
	while (GetTickCount() - start_time < 10000)
	{
		if (_com.read(buff, 1) > 0)
		{
			rx_str = rx_str + buff[0];
		}

		// Motion stopped
		if (rx_str.find("@9 V0") != string::npos)
		{
			return true;
		}

		// Invalid position, unreachable
		if (rx_str.find("E22") != string::npos)
		{
			return false;
		}
	}

	return true;
}

void CuArm::init_com(string comport)
{
	_com.open(comport);
	Sleep(3000);
	set_motion_feedback(true);
}

int CuArm::set_motion_feedback(bool state)
{
	std::string cmd = "M2122 V" + std::to_string(state ? 1 : 0) + "\n";
	send_com(cmd);
	Sleep(500);

	return true;
}

void CuArm::init_robot()
{
	set_pump(_pump);
	set_gripper(_gripper);
	set_pos(_home_pos);
}

bool CuArm::set_pos(Point3f pos)
{
	std::string sendstr = "G0 X" + std::to_string(pos.x) + " Y" + std::to_string(pos.y) + " Z" + std::to_string(pos.z) + " F" + std::to_string(_speed) + " \n";
	send_com(sendstr);
	_com.flush();

	string resp;
	if (get_com(resp) == false)
	{
		return false;
	}

	return true;
}

int CuArm::set_pump(bool state)
{
	if (state != _pump)
	{
		_pump = state;
		std::string cmd = "M2231 V" + std::to_string(state ? 1 : 0) + "\n";
		send_com(cmd);
		_com.flush();
		return true;
	}

	return false;
}

int CuArm::set_gripper(bool state)
{
	if (state != _pump)
	{
		_pump = state;
		std::string cmd = "M2232 V" + std::to_string(state ? 1 : 0) + "\n";
		send_com(cmd);
		_com.flush();
		return true;
	}

	return false;
}
