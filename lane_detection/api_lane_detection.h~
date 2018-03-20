#include "stdio.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"

#include <vector>
#include <iostream>	
#include <iomanip>



using namespace std;
using namespace cv;

#define MAX_NUM_LINES	30
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480
    void centerPoint(Mat &imgGray,Point &center_point,bool is_show_output,Point &carPoint,vector< Point > &vec_left_points,vector< Point > &vec_right_points,double &distance);
void detectLane(Mat &imgGray,Rect roi,string &tmp_name,vector<Point> &vec_match_loc,int &isLane,int &ymin,int &ymax);
void detect_centerPoint(Mat img,Rect roi,string tmp_name);
bool CenterPoint_NII(const Mat& imgGray, Point& centerPos, Point& carPos);
void find_Contours_NII(const Mat& img, vector<vector<Point> > &contours, vector<Vec4i>& hierarchy);
