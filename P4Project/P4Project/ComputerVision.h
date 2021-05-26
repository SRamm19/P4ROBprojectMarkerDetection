#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

//
struct MarkerStruct
{
	// Which marker it is identified as
	int ID; 
	//Marker image location in project
	std::string Location;
	std::vector<int> QRMarkers, QRMarkers90, QRMarkers180, QRMarkers270;
	//Text to show at the marked object
	std::string Text;

	bool shouldBeHighlighted;
	int inPicture;
	cv::Rect position; 
	std::vector<cv::Point3d> MarkerPosition;
	double objectSize, objectPositionX, objectPositionY, objectPositionZ;
};

class ComputerVision
{
public:
	ComputerVision();
	~ComputerVision();
	
	void SetupImages();
	void markerdetection(cv::Mat input, cv::Mat& Output);
	int IsInBorder(int x, int y);

	std::vector<MarkerStruct> MarkerVector;

private:
	void AllocateMarkers();
	MarkerStruct SetQRmarker(MarkerStruct marker, int ID);
	void MarkerValueFinder(cv::Mat input, int Arr[64]);
	int ArrayCompare(int ObservedMarker[]);
	bool MarkerCheck(int ObservedMarker[], std::vector<int> RealMarker);
	void calcPosition(cv::Mat translation, int MarkerID);
	
	std::vector<int> readMarkerImage(cv::Mat image);

	MarkerStruct Marker0;
	MarkerStruct Marker1;
	MarkerStruct Marker2;
	MarkerStruct Marker3;
	MarkerStruct Marker4;
	MarkerStruct Marker5;

	double fx, fy, cx, cy;
	double k1, k2;
	double p1, p2;
	cv::Mat cameraMatrix, distortionCoefficients;
};

