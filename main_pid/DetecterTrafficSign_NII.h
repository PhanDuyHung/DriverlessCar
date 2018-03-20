#pragma once
#include <python3.4/Python.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <stack>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
//standard HSV
#define SCARLAR_LOWER_RED_1 0,50,50//0,100,100//160, 200, 160
#define SCARLAR_UPPER_RED_1 10,255,255//120,255,255//179, 255, 255
#define SCARLAR_LOWER_RED_2 160,50,50////160,100,120//160, 200, 160
#define SCARLAR_UPPER_RED_2 180,255,255//179, 255, 255
//#define SCARLAR_LOWER_RED_3 135,50,50//160, 200, 160
//#define SCARLAR_UPPER_RED_3 145,255,255//179, 255, 255

//#define SCARLAR_LOWER_BLUE_1 90,220,120//75, 128, 128//98,109,20//170, 0, 0
//#define SCARLAR_UPPER_BLUE_1 110,255,155//120, 255, 255//112,255,255//270,255,255

#define SCARLAR_LOWER_BLUE 100, 120, 100//98,109,20//170, 0, 0
#define SCARLAR_UPPER_BLUE 120, 255, 255//112,255,255//270,255,255


#define SCARLAR_LOWER_YELLOW 20,100,100//22, 128, 128
#define SCARLAR_UPPER_YELLOW 30,255,255//38, 255, 255
//
//#define SCARLAR_LOWER_BLACK 0, 0, 200
//#define SCARLAR_UPPER_BLACK 180, 255, 255
//
//#define SCARLAR_LOWER_WHITE 180, 200, 170
//#define SCARLAR_UPPER_WHITE 255, 255, 255

#define SCARLAR_LOWER_BLACK 0, 0, 0
#define SCARLAR_UPPER_BLACK 0, 50, 10

#define SCARLAR_LOWER_WHITE 110, 20, 200//110, 30, 225
#define SCARLAR_UPPER_WHITE 160, 80, 255//160, 88, 250

#define RATIO 1.38
#define SCALAR_SOLID 0, 255, 0
#define THICKNESS 1

#define MINIMUM_SIZE 32
#define MINIMUM_ACCURACY 0.9
class TrafficSign
{
	int id;
	Point center;
	Size size;
	float timeExe;
	
public:	
	TrafficSign(int id = -1,int time = 0, Point c = Point(0, 0), Size s = Size(0,0))
	{
		this->id = id;
		c = center;
		size = s;
		timeExe = time;
	}
	~TrafficSign(){};
	Point getCenTer() const { return center; }
	Size getSize() const { return size; }
	void setCenTer(Point c) { center = c; } 
	void setSize(Size s) { size = s; }
	int getId() const { return id; }
	void setId(int i) { id = i; }
	float getTime() {return timeExe;}
	void DecreaseTime(float delta) {timeExe-=delta;}
};




class Detecter_NII
{
	Detecter_NII();
	static vector<TrafficSign> trafficSigns;
	static PyObject* pyModule;
	static void FindEllipses(const Mat& img, const vector<Point>&contour);
	static void LoadFileSignNames(const string& filename);
	static void DetectTrafficSigns(const Mat& imgSrc);
	static void DrawTrafficSigns(Mat& imgSrc, TrafficSign traffic);
	static Mat CutTrafficSign(const Mat& imgSrc, TrafficSign&);
	static void SetLabel(Mat&, const TrafficSign&, float acc = 0);
public:
	~Detecter_NII();
	static void init();
	static int GetTrafficSignDetected(Mat&);
};

