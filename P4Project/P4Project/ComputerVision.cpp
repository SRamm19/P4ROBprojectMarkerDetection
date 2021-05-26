#include "ComputerVision.h"

ComputerVision::ComputerVision()
{
	Marker0.Location = "figures\\0.png";
	Marker0.Text = "This is marker nr. 0";
	Marker1.Location = "figures\\1.png";
	Marker1.Text = "This is marker nr. 1";
	Marker2.Location = "figures\\2.png";
	Marker2.Text = "This is marker nr. 2";
	Marker3.Location = "figures\\3.png";
	Marker3.Text = "This is marker nr. 3";
	Marker4.Location = "figures\\4.png";
	Marker4.Text = "This is marker nr. 4";
	Marker5.Location = "figures\\5.png";
	Marker5.Text = "This is marker nr. 5";

	fx = 1.241423656749491e+03, fy = 1.338249678764009e+03, cx = 799.726234463772, cy = 692.745483189708;
	k1 = -0.024743933828695, k2 = -0.073477045230117;
	p1 = 0, p2 = 0;
	cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);   // Here we define the camera and dist matrix
	distortionCoefficients = (cv::Mat1d(1, 4) << k1, k2, p1, p2);
}

ComputerVision::~ComputerVision()
{
}

void ComputerVision::SetupImages()
{
	AllocateMarkers();
}

void ComputerVision::AllocateMarkers()
{
	MarkerVector.push_back(SetQRmarker(Marker0, 0));
	MarkerVector.push_back(SetQRmarker(Marker1, 1));
	MarkerVector.push_back(SetQRmarker(Marker2, 2));
	MarkerVector.push_back(SetQRmarker(Marker3, 3));
	MarkerVector.push_back(SetQRmarker(Marker4, 4));
	MarkerVector.push_back(SetQRmarker(Marker5, 5));
}

MarkerStruct ComputerVision::SetQRmarker(MarkerStruct marker, int ID)
{
	marker.ID = ID;
	cv::Mat markerPic = cv::imread(marker.Location, cv::IMREAD_GRAYSCALE);
	cv::threshold(markerPic, markerPic, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	cv::resize(markerPic, markerPic, cv::Size(8, 8));

	marker.QRMarkers = readMarkerImage(markerPic);
	cv::rotate(markerPic, markerPic, cv::ROTATE_90_CLOCKWISE);
	marker.QRMarkers90 = readMarkerImage(markerPic);
	cv::rotate(markerPic, markerPic, cv::ROTATE_90_CLOCKWISE);
	marker.QRMarkers180 = readMarkerImage(markerPic);
	cv::rotate(markerPic, markerPic, cv::ROTATE_90_CLOCKWISE);
	marker.QRMarkers270 = readMarkerImage(markerPic);

	double MarkerLength = 4.2;
	marker.MarkerPosition = {cv::Point3d(-MarkerLength / 2,MarkerLength / 2,0),cv::Point3d(MarkerLength / 2,MarkerLength / 2,0),cv::Point3d(MarkerLength / 2,-MarkerLength / 2,0),cv::Point3d(-MarkerLength / 2,-MarkerLength / 2,0)};

	marker.objectSize = 10.0;
	marker.objectPositionZ = 5.0;

	return marker;
}

std::vector<int> ComputerVision::readMarkerImage(cv::Mat image)
{
	std::vector<int> intVec;
	for (int x = 0; x < image.cols; x++)
	{
		for (int y = 0; y < image.rows; y++)
		{
			intVec.push_back((int)image.at<uchar>(cv::Point(x, y)));
		}
	}
	return intVec;
}

void ComputerVision::markerdetection(cv::Mat input, cv::Mat& Output)
{
	cv::cvtColor(input, input, cv::COLOR_RGB2GRAY);
	cv::Mat Out1 = input.clone();

	cv::GaussianBlur(Out1, Out1, cv::Size(3, 3), 3, 0);
	cv::Canny(Out1, Out1, 25, 75);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(Out1, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Rect>boundRect(contours.size());

	int width = 300;
	int height = 300;
	std::vector<std::vector<cv::Point2f>>  conPoly(contours.size());

	for (int i = 0; i < contours.size(); i++) 
	{
		if (cv::contourArea(contours[i]) > 1000)// && cv::contourArea(contours[i]) < 20000)
		{
			cv::approxPolyDP(contours[i], conPoly[i], cv::arcLength(contours[i], true) * 0.02, true);
		}
	}

	for (int i = 0; i < conPoly.size(); i++) 
	{
		if (conPoly[i].size() == 4)
		{
			cv::Point2f Poly2[4] = {cv::Point2f(conPoly[i][0].x,conPoly[i][0].y),
									cv::Point2f(conPoly[i][1].x,conPoly[i][1].y),    //Finder fire punkter på billedet. 
									cv::Point2f(conPoly[i][2].x,conPoly[i][2].y),
									cv::Point2f(conPoly[i][3].x,conPoly[i][3].y)};

			double imWidth = 300;
			double imHeight = 300;

			cv::Point2f Points[4] = {cv::Point2f(0,0),
									 cv::Point2f(imWidth, 0),
									 cv::Point2f(imWidth, imHeight),      //Laver destination.    
									 cv::Point2f(0, imHeight)};
			cv::Mat Out;
			cv::Mat Matrix = cv::getPerspectiveTransform(Poly2, Points);   //laver transformations matrice 
			//cout << "Matrix" << Matrix << endl;
			cv::warpPerspective(input, Out, Matrix, cv::Size(width, height));  //laver warp perspective. 
			cv::threshold(Out, Out, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
			cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(33, 33));
			cv::morphologyEx(Out, Out, cv::MORPH_OPEN, element);
			cv::morphologyEx(Out, Out, cv::MORPH_CLOSE, element);

			cv::Mat One = Out.clone();

			int Markers1[64] = { 0 };
			//cv::imshow("currentRead", Out);
			MarkerValueFinder(One, Markers1);
			
			int markerID = ArrayCompare(Markers1);
			if (markerID != -1)
			{
				cv::drawContours(Output, contours, i, 255, 2, 8);
				if (MarkerVector[markerID].shouldBeHighlighted)
				{
					boundRect[i] = cv::boundingRect(conPoly[i]);
					cv::rectangle(Output, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 5);
					MarkerVector[markerID].position = boundRect[i];
					cv::putText(Output, MarkerVector[markerID].Text, { boundRect[i].x, boundRect[i].y - 5 }, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 69, 255), 1.5);

					cv::Mat rvec, tvec;
					cv::solvePnP(MarkerVector[markerID].MarkerPosition, conPoly[i], cameraMatrix, distortionCoefficients, rvec, tvec, 1, cv::SOLVEPNP_IPPE_SQUARE);  //Find the transformation
					cv::drawFrameAxes(Output, cameraMatrix, distortionCoefficients, rvec, tvec, 0.5, 2);   //We draw the axis but how we draw something like a box?
					double xVal = tvec.at<double>(0, 0);
					double yVal = tvec.at<double>(1, 0);
					double zVal = tvec.at<double>(2, 0);
					std::string stringVal = std::to_string(xVal) + ", " + std::to_string(yVal) + ", " + std::to_string(zVal);
					cv::putText(Output, stringVal, { boundRect[i].x, boundRect[i].y + boundRect[i].height + 20}, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 10, 255), 1.7);
					calcPosition(tvec, markerID);
					double Magnitude = sqrt(pow(xVal, 2) + pow(yVal, 2) + pow(zVal, 2));
					//std::cout << "translation" << tvec.at<float>(0) << std::endl;
					std::cout << "Marker " << MarkerVector[markerID].ID << "'s magnitude is: " << Magnitude << std::endl;
				}
			}
		}
	}

	for (int i = 0; i < MarkerVector.size(); i++)
	{
		MarkerVector[i].inPicture--;
	}
}

int ComputerVision::ArrayCompare(int ObservedMarker[])
{
	bool state = false;
	for (int j = 0; j < MarkerVector.size(); j++)
	{
		if (MarkerCheck(ObservedMarker, MarkerVector[j].QRMarkers))
		{
			state = true;
		}
		else if (MarkerCheck(ObservedMarker, MarkerVector[j].QRMarkers90))
		{
			state = true;
		}
		else if (MarkerCheck(ObservedMarker, MarkerVector[j].QRMarkers180))
		{
			state = true;
		}
		else if (MarkerCheck(ObservedMarker, MarkerVector[j].QRMarkers270))
		{
			state = true;
		}

		if (state)
		{
			if (MarkerVector[j].inPicture < 1 && MarkerVector[j].shouldBeHighlighted)
			{
				std::cout << "Marker " << j << " detected" << std::endl;
			}
			MarkerVector[j].inPicture = 20;
			return j;
		}
	}

	return -1;
}

bool ComputerVision::MarkerCheck(int ObservedMarker[], std::vector<int> RealMarker)
{
	int k = 0;
	int buffer = 0, bufferMax = 1;
	for (int i = 0; i < size(RealMarker); i++) {
		if (ObservedMarker[i] == RealMarker[i])
			k++;
		else if (buffer < bufferMax)
			buffer++;
		else
			break;
	}

	if (k >= size(RealMarker) - bufferMax)
		return 1;
	else
		return 0;
}

void ComputerVision::MarkerValueFinder(cv::Mat input, int Arr[64])
{
	cv::resize(input, input, cv::Size(8, 8));
	//cv::threshold(input, input, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	for (int x = 0, i = 0; x < input.cols; x++)
	{
		for (int y = 0; y < input.rows; y++, i++)
		{
			Arr[i] = (int)input.at<uchar>(cv::Point(x, y));
		}
	}
}

int ComputerVision::IsInBorder(int x, int y)
{
	for (int i = 0; i < MarkerVector.size(); i++)
	{
		if (MarkerVector[i].inPicture > 0)
		{
			if (x >= MarkerVector[i].position.tl().x && x <= MarkerVector[i].position.br().x && 
				y >= MarkerVector[i].position.tl().y && y <= MarkerVector[i].position.br().y)
			{
				return i;
			}
		}
	}
	return -1;
}

void ComputerVision::calcPosition(cv::Mat translation, int MarkerID)
{
	//Some calculation
	//MarkerVector[MarkerID].objectPositionX = translation.at<double>(0)
	//MarkerVector[MarkerID].objectPositionY = translation.at<double>(0)
}

 