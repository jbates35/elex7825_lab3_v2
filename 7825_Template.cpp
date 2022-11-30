////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated September 26, 2022
////////////////////////////////////////////////////////////////
#include "stdafx.h"

using namespace std;
using namespace cv;

using namespace dnn;
using namespace aruco;

#include <opencv2/face/facemark.hpp>

// Add simple GUI elements
#define CVUI_DISABLE_COMPILATION_NOTICES
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "Robot.h"
#include "Camera.h"

void lab1()
{
   CCamera _virtualcam;
   _virtualcam.createChArUcoBoard();
}

void lab2()
{
   CCamera _virtualcam;
   _virtualcam.calibrate_board();
}

void lab3(int cam_id)
{
  char exit_key = -1;
  CRobot robot;

  
  robot.create_simple_robot();

  while (exit_key != 'q')
  {
    robot.draw_simple_robot();
    exit_key = waitKey(10);
  }
}

void lab4(int cam_id)
{
   char exit_key = -1;
   CRobot robot;
   robot.set_worldview();
   
   robot.create_simple_robot();

   while (exit_key != 'q') {
      robot.draw_more_complex_robot();
      exit_key = waitKey(5);   
   }
}

void lab5(int cam_id)
{
    char exit_key = -1;
    CRobot robot(5);
    robot.set_worldview();

    while (exit_key != 'q')
    {
        robot.create_lab5();
        robot.draw_lab5 ();

        exit_key = waitKey(10);
    }
}

void lab6(int cam_id)
{
   char exit_key = -1;
   CRobot robot(6);
   robot.set_worldview();

   while (exit_key != 'q') {
      robot.create_lab5();
      robot.draw_lab6();

      exit_key = waitKey(10);
   }
  
}

void lab7(int cam_id)
{
   char exit_key = -1;
   CRobot robot(7);
   robot.set_worldview();

   while (exit_key != 'q') {
      robot.create_lab5();
      robot.draw_lab7();

      exit_key = waitKey(10);
   }
}

int main(int argc, char* argv[])
{
  int sel = -1;
  int cam_id = 0;

  while (sel != 0)
  {
    cout << "\n*****************************************************";
    cout << "\n(3) Lab 3 - Virtual Camera";
    cout << "\n(4) Lab 4 - Camera Calibration";
    cout << "\n(5) Lab 5 - Forward Kinematics";
    cout << "\n(6) Lab 6 - Inverse Kinematics";
    cout << "\n(7) Lab 7 - Trajectories";
    cout << "\n(0) Exit";
    cout << "\n\n(1) Create Charuco board";
    cout << "\n(2) Calibrate Camera";
    cout << "\n>> ";

    cin >> sel;
    switch (sel)
    {
    case 1: lab1(); break;
    case 2: lab2(); break;
    case 3: lab3(cam_id); break;
    case 4: lab4(cam_id); break;
    case 5: lab5(cam_id); break;
    case 6: lab6(cam_id); break;
    case 7: lab7(cam_id); break;
    }
  }

  return 1;
}
