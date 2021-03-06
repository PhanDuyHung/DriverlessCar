/**
	This code runs our car automatically and log video, controller (optional)
	Line detection method: Canny
	Targer point: vanishing point
	Control: pca9685

	You should understand this code run to image how we can make this car works from image processing coder's perspective.
	Image processing methods used are very simple, your task is optimize it.
	Besure you set throttle val to 0 before end process. If not, you should stop the car by hand.
	In our experience, if you accidental end the processing and didn't stop the car, you may catch it and switch off the controller physically or run the code again (press up direction button then enter).
	**/
#include "openni2.h"
#include "../openni2/Singleton.h"
#include "DetecterTrafficSign_NII.h"
#include "api_kinect_cv.h"
// api_kinect_cv.h: manipulate openNI2, kinect, depthMap and object detection
#include "api_lane_detection.h"
// api_lane_detection.h: manipulate line detection, finding lane center and vanishing point
#include "api_i2c_pwm.h"
#include "api_uart.h"
#include <iostream>
#include <queue>
#include<math.h>
#include<thread>
#include<mutex>
#include<condition_variable>
#include"myfuzzy.h"
#include "Hal.h"
using namespace openni;
using namespace framework;
using namespace EmbeddedFramework;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480

#define SW1_PIN	164 //160
#define SW2_PIN	163//161
#define SW3_PIN	161 //163
#define SW4_PIN	160 
#define SENSOR	165

int set_throttle_val = 0;
int dir = 0, throttle_val = 0;
int current_state = 0;
PCA9685 *pca9685 = new PCA9685();
Point carPosition, centerPosition(-1,-1);
mutex m1;
Point getpoint(Point p = Point(0, 0)){
	lock_guard<mutex> lock(m1);
	if (p == Point(0, 0)){
		return centerPosition;
	}
	else{
		centerPosition = p;
		return Point(0, 0);
	}
}
int DetectObsDepth(const Mat& depth_img)
{
	int nguong = 50;
	float w = 640, h =480; 
	float roi_width = w/2 - 10, roi_height = h * 0.25;
	cv::Rect roi_left = cv::Rect(w/2, h * 0.20,
	roi_width, roi_height);
	cv::Rect roi_right = cv::Rect(0, h * 0.20,
	roi_width, roi_height);
	Mat roi_img_left,roi_img_right;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	int size= 0;
	int min_area = 18000;
	int rs = 0;
	//Point pl(-1,roi_height/2),pr(-1,roi_height/2);
	vector<cv::Point> contour_left, contour_right;
	//imshow("depth",depth_img);
	//get object
	roi_img_left = depth_img(roi_left).clone();
	threshold(roi_img_left,roi_img_left,nguong,255,THRESH_BINARY_INV);
	imshow("left",roi_img_left);
	findContours(roi_img_left, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	size = contours.size();
	//int minx = w ,maxx = 0;
	int max_left = 0, max_right = 0;
	for (int i = 0; i<size;i++)
	{
		int a = (int)cv::contourArea(contours[i]);
		cout <<"left "<<a<<endl;
		if(a > min_area && a > max_left)
			{
				max_left = a;
				contour_left = contours[i];
			}
	}
		
	roi_img_right = depth_img(roi_right).clone();
	threshold(roi_img_right,roi_img_right,nguong,255,THRESH_BINARY_INV);
	imshow("right",roi_img_right);
	findContours(roi_img_right, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	size = contours.size();
	for (int i = 0; i<size;i++)
	{
		int a = (int)cv::contourArea(contours[i]);
		cout <<"right "<<a<<endl;
		if(a > min_area && a > max_right)
			{
				max_right = a;
				contour_right = contours[i];
			}
	}
	
	if (!contour_left.size() && !contour_right.size()) return 0;
	return (max_left > max_right)? -1 : 1;
}

void Preprocessing_img(Mat& img)
{
	normalize(img, img, 0, 255, NORM_MINMAX);
	//khử nhiễu
	medianBlur(img, img, 5);
	//xóa đối tượng nhỏ
	GaussianBlur(img, img, Size(5, 5), 0);
	//tách biên
	//Canny(img, img, 30, 150);
	
	//phóng to để nối liền ảnh
	Mat kernel = getStructuringElement(MORPH_RECT, Size(2, 2));
	Mat dilation; dilate(img, dilation, kernel);

	threshold(dilation, img, 127, 255, CV_THRESH_BINARY);
	//threshold(img, img, 127, 255, CV_THRESH_BINARY);
}

int DetectObs(Mat& img)
{
	//imshow("img",img);
	//int nguong = 127;
	int w = 640, h =480; 
	int roi_width = w/2, roi_height = h * 0.25;
	cv::Rect roi_left = cv::Rect(0, h * 0.30,
	roi_width, roi_height);
	cv::Rect roi_right = cv::Rect(w/2, h * 0.30,
	roi_width, roi_height);
	Mat roi_img_left,roi_img_right;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	int size= 0;
	int min_area = 18000;
	int rs = 0;
	int max_left = 0, max_right = 0;
	vector<cv::Point> contour_left, contour_right;
	//get object
	roi_img_left = img(roi_left).clone();
	
	// //threshold(roi_img_left,roi_img_left,nguong,255,THRESH_BINARY_INV);
	Preprocessing_img(roi_img_left);
	//imshow("left",roi_img_left);
	findContours(roi_img_left, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	size = contours.size();
	
	for (int i = 0; i<size;i++)
	{
		int a = (int)cv::contourArea(contours[i]);
		if(a > min_area && a > max_left)
			{
				max_left = a;
				contour_left = contours[i];
			}
	}
		
	roi_img_right = img(roi_right).clone();
	//threshold(roi_img_right,roi_img_right,nguong,255,THRESH_BINARY_INV);
	Preprocessing_img(roi_img_right);
	//imshow("right",roi_img_right);
	findContours(roi_img_right, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	size = contours.size();
	for (int i = 0; i<size;i++)
	{
		int a = (int)cv::contourArea(contours[i]);
		if(a > min_area && a > max_right)
			{
				max_right = a;
				contour_right = contours[i];
			}
	}
	cout <<"left:"<<max_left <<" right:"<<max_right<<endl;
	if (!contour_left.size() && !contour_right.size()) return 0;
	return (max_left > max_right)? -1 : 1;
}

/// Get depth Image or BGR image from openNI device
/// Return represent character of each image catched
char analyzeFrame(const VideoFrameRef& frame, Mat& depth_img, Mat& color_img) {
	DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;

    //int w = frame.getWidth();
    //int h = frame.getHeight();
	int w = 640, h =480; 
    depth_img = Mat(h, w, CV_16U);
    Mat depth_img_8u;
    color_img = Mat(h, w, CV_8UC3);
	
	
    switch (frame.getVideoMode().getPixelFormat())
    {
   	case PIXEL_FORMAT_DEPTH_1_MM: return 'm';
    case PIXEL_FORMAT_DEPTH_100_UM:
        depth_img_data = (DepthPixel*)frame.getData();
        memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));
		
        normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
		//normalize(depth_img, depth_img, 255, 0, NORM_MINMAX);

        depth_img_8u.convertTo(depth_img_8u, CV_8U);
		//depth_img.convertTo(depth_img, CV_8U);
		
        depth_img = depth_img_8u;
		// imshow("depth",depth_img);

		
				
		// return obs;
		return 'd';
        break;
    case PIXEL_FORMAT_RGB888:
        color_img_data = (RGB888Pixel*)frame.getData();

        memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));

        cvtColor(color_img, color_img, COLOR_RGB2BGR);
		return 'c';
        break;
		
    default:
        printf("Unknown format\n");
		return 'u';
    }
}
//
void clear(queue<double> &q)
{
	queue<double> empty;
	swap(q, empty);
}
double et = 0, tong = 0;
double dieukhienpid(double in){
	in = in / 40;
	double kp = 0, kd = 0, ki = 0;
	double theta = in*kp + kd*(in - et) + tong;
	et = in;
	//tong += in*ki;
	return theta;
}
//double dieukhienpid(double in){
//	double kp = 0.48, kd = 0, ki = 0.05;
//	double theta = in*kp + kd*(in - et) + tong;
//	et = in;
//	tong += in*ki;
//	return theta;
//}
/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point car, Point dst) {
	double goc;
	if (dst.x == car.x) return 0;
	//if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
	double pi = acos(-1.0);
	double dx = dst.x - car.x;
	double dy = car.y - dst.y;// + 40; // image coordinates system: car.y > dst.y
	if (dx < 0){
		goc = -atan(-dx / dy) * 180 / pi;
	}
	else{
		goc = atan(dx / dy) * 180 / pi;
	}
	return goc;
}
double prv_e = 0;
double getTheta_me(Point car, Point dst)
{
	double alpha, theta;
	if (dst.x == car.x) theta = 0;
	//if (dst.y == car.y) alpha = (dst.x < car.x ? -90 : 90);
	else
	{
		double L = 25, ld = 50;
		double pi = acos(-1.0);
		double dx = dst.x - car.x;
		double dy = car.y - dst.y + ld; // image coordinates system: car.y > dst.y
		// if (dx < 0) alpha = -atan(-dx / dy) * 180 / pi;
		// else alpha = atan(dx / dy) * 180 / pi;

		if (dx < 0) theta = -atan((2 * L / ld)*(-dx / sqrt(dx*dx + dy*dy))) * 180 / pi;

		else theta = atan((2 * L / ld)*(dx / sqrt(dx*dx + dy*dy))) * 180 / pi;
		//PID
		double kP = 0.01, kI = 0.02;
		// theta=theta+kP*dx+kI*(dx-prv_e);
		prv_e = dx;
		//
	}
	return theta;
}

//double getTheta(Point car, Point dst) {
//
//	double kp = 0.56, kd=0.14,ki=0.56;
//	if (dst.x == car.x) return 0;
//	if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
//	double pi = acos(-1.0);
//	double dx = dst.x - car.x;//dx<0 xe dang ben phai duong goc tra ve se am
//	double theta = dx*kp+kd*(dx-et)+tong;
//	et = dx;
//	tong += dx*ki;
//	if (theta > 200) theta = 200;
//	if (theta < -200) theta = -200;
//	return theta;
//}

int cport_nr; // port id of uart.
char buf_send[BUFF_SIZE]; // buffer to store and recive controller messages.

char buf_recv[BUFF_SIZE];

/// Write speed to buffer
void setThrottle(int speed) {
	// speed=60;
	if (speed >= 0)
		sprintf(buf_send, "f%d\n", speed);
	else {
		speed = -speed;
		sprintf(buf_send, "b%d\n", speed);
	}
}
double distance(){
	int n, kc = BUFF_SIZE, giatri = 0, chot = 0;
	if (cport_nr == -1) {
		cerr << "Error: Canot Open ComPort";
		return -1;
	}
	n = api_uart_read(cport_nr, buf_recv);
	if (n > 0)
	{
		buf_recv[n] = 0;
	}
	for (int i = 0; i < BUFF_SIZE; i++){
		if (buf_recv[i] == 'e'){
			kc = i;
		}
		if (i>kc){
			if (buf_recv[i] >= '0'&&buf_recv[i] <= '9')
				giatri = giatri * 10 + (buf_recv[i] - '0');
			else {
				kc = BUFF_SIZE;
				chot = giatri;
				giatri = 0;
			}

		}
		buf_recv[i] = 0;
	}
	return (double)chot * 117 / 20000;
}

double dieukhienmo(double in1){
	static HamThuoc *hamthuoc[8];
	hamthuoc[0] = new hamhinhthangtrai(-55, -40);
	hamthuoc[1] = new hamtamgiac(-55, -40, -30);
	hamthuoc[2] = new hamtamgiac(-40, -30, -20);
	hamthuoc[3] = new hamtamgiac(-30, -20, -5);
	hamthuoc[4] = new hamtamgiac(5, 20, 30);
	hamthuoc[5] = new hamtamgiac(20, 30, 40);
	hamthuoc[6] = new hamtamgiac(30, 40, 55);
	hamthuoc[7] = new hamhinhthangphai(40, 55);

	static ThuocTinh saiso(8, hamthuoc);
	static ThuocTinh tt[1] = { saiso };
	static double	maxvalue[8] = { 200, 180, 130, 80, -80, -130, -180, -200 };
	//static double	maxvalue[8] = { 200, 170, 120, 70, -70, -120, -170, -200 };
	static MyFuzzy myfuzzy(tt, maxvalue);
	//myfuzzy.GetTable(57,35);
	static double vValue;
	vValue = myfuzzy.getvalue(in1);
	return vValue;
}

void dieukhienxe(){
	static double oldthrottval = 0;
	double newtheta;
	Point p = getpoint();
	double angDiff = getTheta(carPosition, getpoint());
	newtheta = dieukhienmo(angDiff);
	//double theta = (-angDiff * 3);
	int pwm2 = api_set_STEERING_control(pca9685, newtheta);
	throttle_val = dieukhienpid(abs(newtheta));
	throttle_val = set_throttle_val - throttle_val;
	if (abs(throttle_val - oldthrottval) > 2){
		oldthrottval = throttle_val;
		api_set_FORWARD_control(pca9685, throttle_val);
		api_uart_write(cport_nr, buf_send);
	}

	/*if (theta>-50 && theta<50){
		throttle_val = 40;
		api_set_FORWARD_control(pca9685, throttle_val);
		api_uart_write(cport_nr, buf_send);
		}
		else if (theta>-120 && theta<120)
		{
		throttle_val = 40;
		api_set_FORWARD_control(pca9685, throttle_val);
		api_uart_write(cport_nr, buf_send);
		}
		else{
		throttle_val = 40;
		api_set_FORWARD_control(pca9685, throttle_val);

		api_uart_write(cport_nr, buf_send);

		}*/
}

bool ready = false;
mutex m;
condition_variable cd;
void dieukhien(){
	int kc;
	while (1){
		unique_lock<mutex> lk(m);
		cd.wait(lk, []{return ready; });
		dieukhienxe();
		ready = false;
	}
}

///////// utilitie functions  ///////////////////////////

int main(int argc, char* argv[]) {
	GPIO *gpio = new GPIO();
	int sw1_stat = 0;
	int sw2_stat = 0;
	int sw3_stat = 0;
	int sw4_stat = 0;
	int sensor = 1;

	// Setup input
	gpio->gpioExport(SW1_PIN);
	gpio->gpioExport(SW2_PIN);
	gpio->gpioExport(SW3_PIN);
	gpio->gpioExport(SW4_PIN);
	gpio->gpioExport(SENSOR);

	gpio->gpioSetDirection(SW1_PIN, INPUT);
	gpio->gpioSetDirection(SW2_PIN, INPUT);
	gpio->gpioSetDirection(SW3_PIN, INPUT);
	gpio->gpioSetDirection(SW4_PIN, INPUT);
	gpio->gpioSetDirection(SENSOR, INPUT);
	usleep(10000);
//tf
	Detecter_NII::init();
	/// Init openNI ///
	Status rc;
	Device device;

	VideoStream depth, color;
	rc = OpenNI::initialize();
	if (rc != STATUS_OK) {
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 0;
	}
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK) {
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 0;
	}
	if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
	    rc = depth.create(device, SENSOR_DEPTH);
	    if (rc == STATUS_OK) {
	    	VideoMode depth_mode = depth.getVideoMode();
	       	depth_mode.setFps(30);
	      	depth_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
	       	depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
	        depth.setVideoMode(depth_mode);

	        rc = depth.start();
	       if (rc != STATUS_OK) {
	            printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
	        }
	    }
	    else {
	        printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
	    }
	}

	if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
		rc = color.create(device, SENSOR_COLOR);
		if (rc == STATUS_OK) {
			VideoMode color_mode = color.getVideoMode();
			color_mode.setFps(30);
			color_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
			color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
			color.setVideoMode(color_mode);

			rc = color.start();
			if (rc != STATUS_OK)
			{
				printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
			}
		}
		else {
			printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
		}
	}

	VideoFrameRef frame;
	VideoStream* streams[] = { &depth, &color };
	// /// End of openNI init phase ///


	 Mat depthImg, colorImg, grayImage;
	
	double theta = 0;

	char key = 0;

	bool hasTraffic = false;
	float timeTurnHasTraffic = 25;;;
	//=========== Init  =======================================================

	////////  Init PCA9685 driver   ///////////////////////////////////////////


	api_pwm_pca9685_init(pca9685);

	if (pca9685->error >= 0)
		api_set_STEERING_control(pca9685, theta);

	/////////  Init UART here   ///////////////////////////////////////////////

	std::thread first(dieukhien);
	if (cport_nr == -1) {
		cerr << "Error: Canot Open ComPort";
		return -1;
	}

	////////  Init direction and ESC speed  ///////////////////////////
	dir = DIR_REVERSE;
	throttle_val = 0;
	theta = 0;
	// int nguong;
    // if (argc == 3) 
	// {nguong = atoi(argv[2]);
	// set_throttle_val = atoi(argv[1]);
	// }
	// Argc == 2 eg ./test-autocar 27 means initial throttle is 27
	if (argc == 2) set_throttle_val = atoi(argv[1]);
	fprintf(stderr, "Initial throttle: %d\n", set_throttle_val);
	
	//getpoint(Point(video_VIDEO_FRAME_WIDTH / 2, video_frame_height * 7 / 8));
	//int frame_width = VIDEO_FRAME_WIDTH;
	//int frame_height = VIDEO_FRAME_HEIGHT;

	//Point prvPosition(VIDEO_FRAME_WIDTH / 2 - 14, VIDEO_FRAME_HEIGHT * 7 / 8);
	//============ End of Init phase  ==========================================

	//============  PID Car Control start here. You must modify this section ===

	bool running = false, started = false, stopped = false;

	double st = 0, et = 0, fps = 0;
	double freq = getTickFrequency();

	bool is_show_cam = true;

	int countF = 0;
	queue<double> Q;
	double S = 0;
	double newtheta = 0;
	double distanceLane = VIDEO_FRAME_WIDTH / 2;
	int count = 0;
	bool hasSensor = false;
	int hasObs = 0;
	int timeObs = 20;
	unsigned int sensor_status = 1;
	unsigned int sensor_old = 1;
	unsigned int bt_val[2], bt_old_val[2];

	std::vector<TrafficSign> vecTraffic;
	while (true)
	{
		Point tmp_point(0, 0);
		st = getTickCount();
		key = getkey();

		/*gpio->gpioGetValue(SW1_PIN, &bt_val);
		if (!bt_val) {
			if (bt_old_val) {
				key = 's';
				bt_old_val = 0;
			}
		} else bt_old_val = 1;
		gpio->gpioGetValue(SW3_PIN, &bt_val);
		if (!bt_val) {
			if (bt_old_val) {
				key = 'f';
				bt_old_val = 0;
			}
		} else bt_old_val = 1;*/
		gpio->gpioGetValue(SW1_PIN, &bt_val[0]);
		gpio->gpioGetValue(SW3_PIN, &bt_val[1]);
		if (bt_val[0] == 0 && bt_old_val[0] == 1) {
			key = 's';
		}
		if (bt_val[1] == 0 && bt_old_val[1] == 1) {
			key = 'f';
		}
		bt_old_val[0] = bt_val[0];
		bt_old_val[1] = bt_val[1];
		//cout << "bt1:" << bt_val[0] << endl;

		gpio->gpioGetValue(SENSOR, &sensor_status);



		if (key == 's') {
			distanceLane = VIDEO_FRAME_WIDTH / 2;
			running = !running;
			clear(Q);
			S = 0;
			//  Reset=0;
			tong = 0;
		}
		if (key == 'f') {
			fprintf(stderr, "End process.\n");
			theta = 0;
			throttle_val = 0;
			api_set_FORWARD_control(pca9685, throttle_val);
			api_uart_write(cport_nr, buf_send);
			api_set_STEERING_control(pca9685, theta);
			break;
		}

		if (running) 
		{
			
				//// Check PCA9685 driver ////////////////////////////////////////////

				if (pca9685->error < 0)
				{
					cout << endl << "Error: PWM driver" << endl << flush;
					break;
				}
				if (!started)
				{
					fprintf(stderr, "ON\n");
					started = true; stopped = false;
					throttle_val = set_throttle_val;
					api_set_FORWARD_control(pca9685, throttle_val);
					api_uart_write(cport_nr, buf_send);
				}

				////////  Get input image from camera   //////////////////////////////
				int readyStream = -1;
				rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
				if (rc != STATUS_OK)
				{
					printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
					break;
				}
				
				switch (readyStream)
				{
				case 0:
					// Depth
					depth.readFrame(&frame);
					break;
				case 1:
					// Color
					color.readFrame(&frame);
					break;
				default:
					printf("Unxpected stream\n");
				}
//NII ne vat can
				// if (hasObs != 0 && timeObs > 0)
				// {
				// 	timeObs--;
				// 	if (set_throttle_val>0) throttle_val = set_throttle_val - 10;
					
				// 	if (hasObs == -1)
				// 		theta = -90;
					
				// 	else 
				// 		theta = +90;
						
				// 	cout <<"bs:"<<hasObs<<endl;
				// 	//cout <<theta<<endl;
				// 	api_set_STEERING_control(pca9685, theta);
				// 	api_set_FORWARD_control(pca9685, throttle_val);
				// 	et = getTickCount();
				// 	fps = 1.0 / ((et - st) / freq);
				// 	continue;
				// }
				char recordStatus = analyzeFrame(frame, depthImg, colorImg);
				//imshow("color", colorImg);
				//imshow("depth", depthImg);
				if (recordStatus == 'd') 
				{
					// //imshow("depth", depthImg);
					// hasObs = DetectObsDepth(depthImg);
					// cout << "obs:"<<hasObs<<endl;
					// if (hasObs != 0)
					// {
					// 	timeObs = 20;
					// 	continue;
					// }
				}
			
				////////// Detect Center Point ////////////////////////////////////

				if (recordStatus == 'c') {
					/**
						Status = BGR Image. Since we developed this project to run a sample, we only used information of gray image.
						**/
					flip(colorImg, colorImg, 1);
					cvtColor(colorImg, grayImage, CV_BGR2GRAY);
					// if (!sensor_status)
					// {
						// hasObs = DetectObs(grayImage);
						// if (hasObs != 0) 
						// {
						// 	timeObs = 20;
						// 	continue;
					 	// }
					// }
//NII chay xe	
					//NII - Detect Traffic Sign
					int labelTraffic = Detecter_NII::GetTrafficSignDetected(colorImg);
					//cout << "label: " <<labelTraffic<<endl;
					
					if (labelTraffic != -1)
					{
						hasTraffic = true;
						if (set_throttle_val > 0) throttle_val = set_throttle_val -10;
						if (labelTraffic == 1) //turn left
						{
							theta = -90;
						}
						else if (labelTraffic == 0)//turn right
						{
							theta = 90;
						}	
					}


					if (hasTraffic)
						if (timeTurnHasTraffic > 0)
						{
							timeTurnHasTraffic--;
							//cout <<"time turn:"<<timeTurnHasTraffic<<endl;
							api_set_STEERING_control(pca9685, theta);
							api_set_FORWARD_control(pca9685, throttle_val);
							et = getTickCount();
							fps = 1.0 / ((et - st) / freq);
							//cout <<"=============================theta: "<<theta<<endl;
							continue;
						}
						else 
						{
							hasTraffic = false;
							timeTurnHasTraffic = 25;
						}
					
					//		get center point of lane
					bool hasCenter = CenterPoint_NII(grayImage, centerPosition,carPosition);
					

					////////// using for debug stuff  ///////////////////////////////////
					if (is_show_cam) {
						if(!grayImage.empty())
							//imshow("gray", grayImage);
								if(!colorImg.empty())
							//imshow("dst", colorImg);
						waitKey(1);
					}
			
					//prvPosition = centerPosition;
					double angDiff;
					bool maxAngle = 0;
					
					if (hasCenter)
						{angDiff = getTheta(carPosition, centerPosition);
						if (throttle_val >= 5) throttle_val = set_throttle_val - 5;
						}
					else
						angDiff = 0;
					int dolon = abs(angDiff);
					if (dolon >= 45) {
						
						if (throttle_val >= 5) throttle_val = set_throttle_val - 5;
						theta = 2.5*angDiff;
						// if (angDiff < 0) theta = -90;
						// else theta = 90;
					}
					else 
					{ 
						if (throttle_val >= 5) throttle_val = set_throttle_val - 5;
						if (dolon >= 30) 
							theta = 2.0*angDiff;
						else
						{
							throttle_val = set_throttle_val;
							theta = 1.8*angDiff;
						} 
					}
					//cout << "throttle: "<<throttle_val<<endl;
					//std::cout << "theta" << theta << std::endl;
					//cout << "angle_drif: " << angDiff << endl;
					theta = -theta;
					api_set_STEERING_control(pca9685, theta);
				}
			
			///////  Your PID code here"  //////////////////////////////////////////
			//cout << "Throttle: " << throttle_val << endl;
			api_set_FORWARD_control(pca9685, throttle_val);
			et = getTickCount();
			fps = 1.0 / ((et - st) / freq);
			//if (abs(theta) > 40) usleep(200000);
			//            cout << "FPS: "<< fps<< "   throttle:"<<throttle_val<< '\n';

			// waitKey(1);
			//if (key == 27) break;
		}
		else
		{
			theta = 0;
			throttle_val = 0;
			if (!stopped)
			{
				fprintf(stderr, "OFF\n");
				stopped = true; started = false;
				api_set_FORWARD_control(pca9685, throttle_val);
				api_uart_write(cport_nr, buf_send);
			}
			api_set_STEERING_control(pca9685, theta);
			//sleep(1);
		}

	}
	//////////  Release //////////////////////////////////////////////////////
	// if(is_save_file)
	//        {
	//        gray_videoWriter.release();
	//        color_videoWriter.release();
	//        //depth_videoWriter.release();
	//        fclose(thetaLogFile);
	// }
	return 0;
}
