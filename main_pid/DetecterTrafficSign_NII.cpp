﻿#include "DetecterTrafficSign_NII.h"

vector<TrafficSign> Detecter_NII::trafficSigns;
PyObject* Detecter_NII::pyModule = NULL;

Detecter_NII::Detecter_NII()
{

}
Detecter_NII::~Detecter_NII()
{
	delete pyModule;
}

void Detecter_NII::init()
{
		// Init python
	Py_Initialize();
	//PySys_SetArgv(argc, argv);

	//Import Module
	pyModule = PyImport_ImportModule("Predict");
	PyErr_Occurred();
	PyErr_Print();
}

void Detecter_NII::FindEllipses(const Mat& img, const vector<Point>&contour) {

	if (contour.size() >= 5)
	{
		RotatedRect temp = fitEllipse(Mat(contour));
		//if (abs(temp.size.width - temp.size.height) > 20) continue;
		//=======loại bỏ các hình không nằm hoàn toàn trong image hoac những hình không phải hình tròn
		if (temp.size.width > temp.size.height) //ellipse nam ngang
		{
			if (temp.size.width / temp.size.height > RATIO) return;
			if (temp.center.x - temp.size.height / 2 < 0 || temp.center.x + temp.size.height / 2 > img.cols || temp.center.y - temp.size.width / 2 < 0 || temp.center.y + temp.size.width / 2 > img.rows) return;
		}
		else
		{
			if (temp.size.height / temp.size.width > RATIO) return;
			if (temp.center.x - temp.size.width / 2 < 0 || temp.center.x + temp.size.width / 2 > img.cols || temp.center.y - temp.size.height / 2 < 0 || temp.center.y + temp.size.height / 2 > img.rows) return;
		}

		TrafficSign traffic;
		traffic.setCenTer(temp.center);
		//int length = (int)(temp.size.width < temp.size.height) ? (int)temp.size.height : (int)temp.size.width;
		traffic.setSize(temp.size);

		if (temp.size.width >= MINIMUM_SIZE && temp.size.height >= MINIMUM_SIZE)
			//push into trafficSigns
			trafficSigns.push_back(traffic);
	}

}

void Detecter_NII::DetectTrafficSigns(const Mat& imgSrc)
{
	int roi_width = 640, roi_height = 480 * 0.3;
	cv::Rect roi = cv::Rect(0, 0,roi_width, roi_height);

	Mat img = imgSrc(roi).clone();

	cvtColor(img, img, CV_BGR2HSV);

	Mat imgBlue;
	inRange(img, Scalar(SCARLAR_LOWER_BLUE), Scalar(SCARLAR_UPPER_BLUE), imgBlue);

	normalize(imgBlue, imgBlue, 0, 255, NORM_MINMAX);
	//khử nhiễu
	medianBlur(imgBlue, imgBlue, 5);
	//xóa đối tượng nhỏ
	Mat blured; GaussianBlur(imgBlue, blured, Size(5, 5), 0);
	//tách biên
	Mat edge; Canny(blured, edge, 30, 150);
	
	//phóng to để nối liền ảnh
	Mat kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
	Mat dilation; dilate(edge, dilation, kernel);

	Mat img_detect;  threshold(dilation, img_detect, 127, 255, CV_THRESH_BINARY);
	// find contours and store them all as a list
	vector<vector<Point> >contours;
	findContours(img_detect, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	int size = contours.size();
	for (int i = 0;i<size;i++)
	{
		/*============CIRCLE===============*/
		FindEllipses(img, contours[i]);
	}
	if (trafficSigns.empty()) return;

	//sap xep theo thu giam dan
	stack<int> s;
	int left = 0, right = (int)trafficSigns.size() - 1;
	s.push(left); s.push(right);
	
	while (!s.empty())
	{
		right = s.top(); s.pop();
		left = s.top(); s.pop();
		int i = left, j = right, k = (i + j) / 2;
		while (i <= j)
		{
			int length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			int length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			int length_k = trafficSigns[k].getSize().width + trafficSigns[k].getSize().height;
			while (length_i < length_k) {
				i++;
				length_i = trafficSigns[i].getSize().width + trafficSigns[i].getSize().height;
			}
			while (length_j > length_k) {
				j--;
				length_j = trafficSigns[j].getSize().width + trafficSigns[j].getSize().height;
			}
			if (i <= j)
			{
				TrafficSign t = trafficSigns[i];
				trafficSigns[i] = trafficSigns[j];
				trafficSigns[j] = t;
				i++; j--;
			}
		}
		if (i < right) {
			s.push(i); s.push(right);
		}
		if (j > left){
			s.push(left); s.push(j);
		}
	}

	//xoa nhung duong tron nam trong duong tron khac
	for (int i = (int)trafficSigns.size() - 1; i > 0; --i) {
		Point center_i = trafficSigns[i].getCenTer();
		//float radius_i = (float)trafficSigns[i].getLength() / 2;
		int radius_i = ((trafficSigns[i].getSize().width > trafficSigns[i].getSize().height) ? trafficSigns[i].getSize().width : trafficSigns[i].getSize().height)/2;
		

		for (int j = i - 1; j >= 0; --j){
			Point center_j = trafficSigns[j].getCenTer();
			int radius_j = ((trafficSigns[j].getSize().width > trafficSigns[j].getSize().height) ? trafficSigns[j].getSize().width : trafficSigns[j].getSize().height) / 2;
			//float radius_j = (float)trafficSigns[j].getLength() / 2;

			double d = sqrt((center_i.x - center_j.x)*(center_i.x - center_j.x) + (center_i.y - center_j.y)*(center_i.y - center_j.y));
			if (d < radius_i + radius_j)
			{
				trafficSigns.erase(trafficSigns.begin() + j);
				i--;
			}
		}
	}
}

void Detecter_NII::DrawTrafficSigns(Mat& imgSrc,TrafficSign traffic)
{
		Point center = traffic.getCenTer();
		Size size = traffic.getSize();
		//tìm tọa độ 4 đỉnh A B C D
		Point A, B, C, D;
		A = Point(center.x - size.width / 2, center.y - size.height / 2);
		B = Point(center.x + size.width / 2, center.y - size.height / 2);
		C = Point(center.x + size.width / 2, center.y + size.height / 2);
		D = Point(center.x - size.width / 2, center.y + size.height / 2);
		//Draw square
		line(imgSrc, A, B, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, B, C, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, C, D, Scalar(SCALAR_SOLID), THICKNESS);
		line(imgSrc, D, A, Scalar(SCALAR_SOLID), THICKNESS);
}

Mat Detecter_NII::CutTrafficSign(const Mat& imgSrc, TrafficSign& traffic) {

	Point c = traffic.getCenTer();
	Size size = traffic.getSize();
	Range x(0, 0), y(0, 0);
	x.start = (c.x - size.width / 2 >= 0) ? c.x - size.width / 2 : 0;
	x.end = (c.x + size.width / 2 <= imgSrc.cols) ? c.x + size.width / 2 : imgSrc.cols;
	y.start = (c.y - size.height / 2 >= 0) ? c.y - size.height / 2 : 0;
	y.end = (c.y + size.height / 2 <= imgSrc.rows) ? c.y + size.height / 2 : imgSrc.rows;

	Mat temp(imgSrc, y, x);
	return temp;
}

int Detecter_NII::GetTrafficSignDetected(Mat& frame)
{
	int result = 0;
	DetectTrafficSigns(frame);
	int size = trafficSigns.size();
	for (int i=0;i<size;i++)
	{
		Mat imgTraffic = CutTrafficSign(frame, trafficSigns[i]);
		imwrite("imageDetected.jpg", imgTraffic);

		//======RECOGNIZE BY PYTHON==========
		PyObject*pyFunc = PyObject_GetAttrString(pyModule, "getPredict");
		PyObject*pyResult = PyObject_CallObject(pyFunc, NULL);
		int id = (int)PyFloat_AsDouble(pyResult);

		pyFunc = PyObject_GetAttrString(pyModule, "getAcc");
		pyResult = PyObject_CallObject(pyFunc, NULL);
		float acc = PyFloat_AsDouble(pyResult);

		if (acc > MINIMUM_ACCURACY)
		{
			trafficSigns[i].setId(id);
			DrawTrafficSigns(frame, trafficSigns[i]);
			result = id;
		}
	}
	trafficSigns.clear();
	return result;
}