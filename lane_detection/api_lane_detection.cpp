#include "api_lane_detection.h"
enum ConvolutionType {
	/* Return the full convolution, including border */
	CONVOLUTION_FULL,

	/* Return only the part that corresponds to the original image */
	CONVOLUTION_SAME,

	/* Return only the submatrix containing elements that were not influenced by the border */
	CONVOLUTION_VALID
};


struct res_contour
{
	//   vector<Point> contour;
	int u;
	double l;
};

bool Compare(const res_contour x, const res_contour y)
{
	return x.l < y.l;
}
double sqr(double x) {
	return x * x;
}

bool inRange(double val, double l, double r) {
	return (l <= val && val <= r);
}
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int detect_obs(Mat &imgGray, Mat &dst, Rect roi)	
{
	Mat img = imgGray(roi).clone();
	GaussianBlur(img, img, Size(11, 11), 0);
	//    Size ksize(9,9);
	//    blur(img, img, ksize );
	Mat img_bin;
	threshold(img, img_bin, 100, 255, CV_THRESH_BINARY);

	Mat mThres_Gray;
	double CannyAccThresh = threshold(img, mThres_Gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	double CannyThresh = 0.1 * CannyAccThresh;
	Mat cannyImg;
	Canny(img, cannyImg, CannyThresh, CannyAccThresh);

	dst = cannyImg.clone();
	cv::imshow("canny obs", dst);
}

void CalDummy(Point& point_dummy,const Point& p,Point& carPos,int roi_width,int roi_height)
{
	carPos.x = roi_width / 2;
	carPos.y = roi_height;
	//cout<<p.x<<endl;
	point_dummy.y = (roi_height * 0.5);
	if (p.x >= 0 && p.x < 100)
	{
		point_dummy.x = (roi_width * 0.90);//0.64
	}
	else if (p.x >= 100 && p.x < 190)
	{
		point_dummy.x = (roi_width * 1);
	}
	else if (p.x >= 190 && p.x < 230)
	{
		point_dummy.x = (roi_width * 0.95);//0.95
	}
	else if (p.x >= 230 && p.x < 280)
	{
		point_dummy.x = (roi_width * 1.3);//0.95
	}
	else if (p.x >= 280 && p.x < 320)
	{
		point_dummy.x = (roi_width * 1.3);//0.95
	}
	else if (p.x > 320 && p.x < 360)
	{
		point_dummy.x = (roi_width * 0.1);
	}
	else if (p.x >= 360 && p.x < 400)
	{
		point_dummy.x = (roi_width * 0.15);
	}
	else if (p.x >= 400 && p.x < 450)
	{
		point_dummy.x = (roi_width * 0.20);
	}
	else if (p.x >= 450 && p.x < 550)
	{
		point_dummy.x = (roi_width * 0.2);
	}
	else if (p.x >= 550 && p.x <= 640)
	{
		point_dummy.x = (roi_width * 0.1);
	}
}

bool CenterPoint_NII_old(const Mat& imgGray, Point& centerPos, Point& carPos)
{
	int frame_width = 640, frame_height = 480;
	int min_area = 1500, max_area = 4000;
	float dummy_not_right = 0.75, dummy_not_left = 0.30;
	int roi_width = frame_width, roi_height = frame_height * 0.20;
	cv::Rect roi = cv::Rect(0, frame_height * 0.75,
		roi_width, roi_height);

	Mat img = imgGray(roi).clone();
	GaussianBlur(img, img, Size(11, 11), 0);

	Mat img_bin;
	threshold(img, img_bin, 220, 255, CV_THRESH_BINARY);

	//Mat img_canny;
	//Canny(img_bin, img_canny, 40, 80);

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	findContours(img_bin, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Mat img_contours;
	cvtColor(img_bin, img_contours, CV_GRAY2BGR);

	// for (size_t i = 0; i < contours.size(); i++)
	// {
	// 	Scalar color = Scalar(0, 255, 255);
	// 	drawContours(img_contours, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
	// }
	
	Point point_dummy(-1, -1);
	//int a0 = (int)cv::contourArea(contours[0]);
	//int a1;
	if (contours.size()==0 || (contours.size() == 1 && ((int)cv::contourArea(contours[0]) <= min_area || (int)cv::contourArea(contours[0]) >= max_area))) return false;//miss 2lanes
	if (contours.size() == 1)//miss 1 lane
	{	
		vector<cv::Point> convex_hull;
		cv::approxPolyDP(cv::Mat(contours[0]), contours[0], 5, true);
		cv::convexHull(contours[0], convex_hull, false);
		// center of gravity
		cv::Moments mo = cv::moments(convex_hull);
		Point p;
		p = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);
		
		CalDummy(point_dummy,p,carPos,roi_width,roi_height);
		circle(img_contours, p, 5, Scalar(255, 0, 0), 2);
		circle(img_contours, point_dummy, 5, Scalar(255, 0, 0), 2);
		centerPos.x = (p.x + point_dummy.x) / 2;
		centerPos.y = (p.y + point_dummy.y) / 2;
		circle(img_contours, centerPos, 5, Scalar(255, 0, 0), 2);
		circle(img_contours, carPos, 5, Scalar(255, 255, 255), 2);
		//imshow("get center", img_contours);
		return true;
	}
	
	//center point
	Point point0(-1, -1), point1(-1, -1);
	
	vector<cv::Point> contour_0;
	vector<cv::Point> contour_1;
	vector<cv::Point> convex_hull_0, convex_hull_1;
	//vector<cv::Point> vec_left, vec_right;
	int max1, max2;
	// find 2 max area contours
	if (contours.size() >= 2)
	{
		int area0 = (int)cv::contourArea(contours[0]);
		int area1 = (int)cv::contourArea(contours[1]);
		if (area0 > area1)
		{
			contour_0 = contours[0];
			contour_1 = contours[1];
		}
		else
		{
			contour_0 = contours[1];
			contour_1 = contours[0];
		}
		max1 = area0;
		max2 = area1;
	}

	for (unsigned int i = 0; i < contours.size(); ++i) {
		int area = (int)cv::contourArea(contours[i]);
		if (area >= max1) {
			contour_1 = contour_0;
			contour_0 = contours[i];
			max2 = max1;
			max1 = area;
		}
		else
		{
			contour_1 = contours[i];
			max2 = area;
		}
	}
	// simplify large contours
	cv::approxPolyDP(cv::Mat(contour_0), contour_0, 5, true);
	cv::approxPolyDP(cv::Mat(contour_1), contour_1, 5, true);

	// convex hull
	cv::convexHull(contour_0, convex_hull_0, false);
	cv::convexHull(contour_1, convex_hull_1, false);
	//if (convex_hull_0.size() < 3) return;
	//if (convex_hull_1.size() < 3) return;

	// center of gravity
	cv::Moments mo = cv::moments(convex_hull_0);
	point0 = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);


	mo = cv::moments(convex_hull_1);
	point1 = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);

	bool isDefaultCarPos = false;
	//miss
	int a0 = (int)cv::contourArea(contour_0);
	int a1 = (int)cv::contourArea(contour_1);
	if ((a0 <= min_area || a0>=max_area) && (a1 <= min_area || a1 >= max_area)) return false;
	if (a0 <= min_area || a0 >= max_area)
	{
		CalDummy(point0,point1,carPos,roi_width,roi_height);
		isDefaultCarPos = true;
	}
	else if(a1 <= min_area || a1>=max_area)
	{
		CalDummy(point1,point0,carPos,roi_width,roi_height);
		isDefaultCarPos = true;
	} 
	centerPos.x = (point0.x + point1.x) / 2;
	centerPos.y = (point0.y + point1.y) / 2;
	//cout << "p0: " << point0.x << " " << point0.y << "  p1: " << point1.x << " " << point1.y << " center: " << centerPos.x << " " << centerPos.y << endl;
	circle(img_contours, point0, 5, Scalar(255, 0, 0), 2);
	circle(img_contours, point1, 5, Scalar(255, 0, 0), 2);
	circle(img_contours, centerPos, 5, Scalar(255, 0, 0), 2);
	//Point carPos; 
	if (!isDefaultCarPos)
	{
		carPos.y = roi_height;
		//car point is center of 2 lines have y maximum
		Point carPos0, carPos1;
		if (point0.x < point1.x) //point0 is left, point1 is right
		{
			int max_x_0 = 0, max_y_0 = 0;
			for (size_t i = 0; i < contour_0.size(); i++)
			{
				if (contour_0[i].y > max_y_0 || (contour_0[i].y == max_y_0 && contour_0[i].x > max_x_0))
				{
					carPos0 = contour_0[i];
					max_x_0 = contour_0[i].x;
					max_y_0 = contour_0[i].y;
				}
			}

			int min_x_1 = img_bin.cols, max_y_1 = 0;
			for (size_t i = 0; i < contour_1.size(); i++)
			{
				if (contour_1[i].y > max_y_1 || (contour_1[i].y == max_y_1 && contour_1[i].x < min_x_1))
				{
					carPos1 = contour_1[i];
					min_x_1 = contour_1[i].x;
					max_y_1 = contour_1[i].y;
				}
			}
		}
		else //point1 is left, point0 is right
		{
			int max_x_1 = 0, max_y_1 = 0;
			for (size_t i = 0; i < contour_1.size(); i++)
			{
				if (contour_1[i].y > max_y_1 || (contour_1[i].y == max_y_1 && contour_1[i].x > max_x_1))
				{
					carPos1 = contour_1[i];
					max_x_1 = contour_1[i].x;
					max_y_1 = contour_1[i].y;
				}
			}

			int min_x_0 = img_bin.cols, max_y_0 = 0;
			for (size_t i = 0; i < contour_0.size(); i++)
			{
				if (contour_0[i].y > max_y_0 || (contour_0[i].y == max_y_0 && contour_0[i].x < min_x_0))
				{
					carPos0 = contour_0[i];
					min_x_0 = contour_0[i].x;
					max_y_0 = contour_0[i].y;
				}
			}
		}

		carPos.x = (carPos0.x + carPos1.x) / 2;
	}
	circle(img_contours, carPos, 5, Scalar(255, 255, 255), 2);
	//imshow("get center", img_contours);
	return true;
}

bool CenterPoint_NII(const Mat& imgGray, Point& centerPos, Point& carPos)
{
	float frame_width = 640, frame_height = 480;
	int min_area = 2000, max_area = 7000;
	float dummy_not_right = 0.75, dummy_not_left = 0.30;
	float roi_width = frame_width, roi_height = frame_height * 0.20;
	cv::Rect roi = cv::Rect(0, frame_height * 0.75,
		roi_width, roi_height);

	Mat img = imgGray(roi).clone();
	GaussianBlur(img, img, Size(11, 11), 0);

	threshold(img, img, 220, 255, CV_THRESH_BINARY);

	//Canny(img, img, 40, 80);

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	Mat img_contours;
	cvtColor(img, img_contours, CV_GRAY2BGR);

	for (size_t i = 0; i < contours.size(); i++)
	{
		Scalar color = Scalar(0, 255, 255);
		drawContours(img_contours, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
	}

	Point point_left(-1,-1),point_right(-1,-1),carPos0,carPos1;
	vector<cv::Point> convex_hull;
	vector<cv::Point> contour;
	int size = contours.size();
	for (size_t i = 0; i < size; i++)
	{
		int area = (int)cv::contourArea(contours[i]);
		//imshow("contours",img_contours);
		//cout <<"area"<<area<<endl;
		if (area <= min_area || area >= max_area) continue;
		
		// simplify large contours
		cv::approxPolyDP(cv::Mat(contours[i]), contour, 5, true);

		// convex hull
		cv::convexHull(contour, convex_hull, false);

		// center of gravity
		cv::Moments mo = cv::moments(convex_hull);
		Point p = cv::Point(mo.m10 / mo.m00, mo.m01 / mo.m00);
		//cout <<"p: "<<p.x<<endl;
		//car Position is center point of 2 points have y max in 2 convex_hulls
		int convex_size = convex_hull.size();
		if (p.x < roi_width/2)
		{
			if (point_left.x == -1 || point_left.x < p.x) point_left = p;
			int max_x_0 = 0, max_y_0 = 0;
			for (size_t j = 0; j < convex_size; j++)
			{
				if (convex_hull[j].y > max_y_0 || (convex_hull[j].y == max_y_0 && convex_hull[j].x > max_x_0))
				{
					carPos0 = convex_hull[i];
					max_x_0 = convex_hull[i].x;
					max_y_0 = convex_hull[i].y;
				}
			}
		}
		else 
		{
			if (point_right.x == -1 || p.x < point_right.x) point_right = p;
			int min_x_1 = roi_width, max_y_1 = 0;
			for (size_t j = 0; j < convex_size; j++)
			{
				if (convex_hull[j].y > max_y_1 || (convex_hull[j].y == max_y_1 && convex_hull[j].x < min_x_1))
				{
					carPos1 = convex_hull[j];
					min_x_1 = convex_hull[j].x;
					max_y_1 = convex_hull[j].y;
				}
			}
		}
	}
	//cout <<"car left:"<<carPos0.x <<" car right: "<<carPos1.x<<endl;
	//cout <<"left:"<<point_left.x <<" right: "<<point_right.x<<endl;
	if (point_left.x != -1 && point_right.x != -1)
		{
			carPos.x = (carPos0.x + carPos1.x) / 2;
			if (carPos.x <roi_width*0.35 || carPos.x > roi_width*0.75)
			carPos.y = roi_height;
		}
	else if (point_left.x == -1 && point_right.x == -1) //miss 2 lanes
		return false;
	if (point_left.x == -1) //not left
		{
			CalDummy(point_left,point_right,carPos,roi_width,roi_height);
		}
	if (point_right.x == -1) //not right
		{
			CalDummy(point_right,point_left,carPos,roi_width,roi_height);
		}
	
	
	centerPos.x = (point_left.x + point_right.x) / 2;
	centerPos.y = (point_left.y + point_right.y) / 2;
	// circle(img_contours, point_left, 5, Scalar(255, 0, 0), 2);
	// circle(img_contours, point_right, 5, Scalar(255, 0, 0), 2);
	// circle(img_contours, centerPos, 5, Scalar(255, 0, 0), 5);
	// circle(img_contours, carPos, 5, Scalar(255, 255, 0), 5);
	//imshow("lane",img_contours);
	return true;
}
