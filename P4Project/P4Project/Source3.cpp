#include <algorithm>
#include <iostream>
#include "ComputerVision.h"
#include "RobotControl.h"

ComputerVision CompVis;
RobotControl ControlRobot;

void CallBackFunc(int event, int x, int y, int flags, void* userdata);

int main(int argc, char** argv) 
{
	CompVis.SetupImages();
	ControlRobot.runDynamixelSetup();

	cv::namedWindow("frames", 1);
	cv::setMouseCallback("frames", CallBackFunc, NULL);	

	cv::VideoCapture camera;
	camera.open(1);
	camera.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	for (int i = 0; i < CompVis.MarkerVector.size(); i++)
	{
		char confirmation;
		std::cout << "Should marker " << i << " be detected?(1/0)" << std::endl;
		std::cin >> confirmation;
		if (confirmation == '1')
		{
			CompVis.MarkerVector[i].shouldBeHighlighted = true;
		}
		else
		{
			CompVis.MarkerVector[i].shouldBeHighlighted = false;
		}
	}


	int frameratecounter = 0;
	while(true)
	{
		cv::Mat frame;
		camera >> frame;
		CompVis.markerdetection(frame, frame);
		cv::resize(frame, frame, cv::Size(640*1.5, 360*1.5));
		cv::imshow("frames", frame);
		frameratecounter++;
		if (cv::waitKey(10) == 27) break;
	}
	std::cout << frameratecounter << std::endl;
	return 0;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	int objectID = CompVis.IsInBorder(x*1.333333, y*1.333333);
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x* 1.333333 << ", " << y* 1.333333 << ")" << std::endl;
		if (objectID != -1)
		{
			std::cout << "You clicked object " << objectID << std::endl;
			ControlRobot.moveGripperToObject(CompVis.MarkerVector[objectID].objectPositionX, 
											 CompVis.MarkerVector[objectID].objectPositionY, 
											 CompVis.MarkerVector[objectID].objectPositionZ, 
											 CompVis.MarkerVector[objectID].objectSize);
		}
	}
} 