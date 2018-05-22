#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <termio.h>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "CqUsbCam.h"
#include "SensorCapbablity.h"

//namespace CCqUsbCam
//{

class cqUsbCamNode
{
  public:
	// ros nodehandle
	ros::NodeHandle node_;

	// image message and publisher
	image_transport::Publisher pub_img;
	sensor_msgs::Image img_msg;

	//parameters
	std::string cameraName_, cameraFrameID_, trigMode_, resolution_;
	int exposure_, gain_, frameRate_;

	// camera handler
	CCqUsbCam cam0, *pCamInUse;

	string sensor = "MT9V034";
	unsigned int g_width = 752;
	unsigned int g_height = 480;

	pthread_mutex_t mutexDisp;
	pthread_mutex_t mutexCam;


	// member functions
	unsigned short hex2dec(char *hex)

	{

		unsigned short number = 0;

		char *p = hex;

		for (p = hex; *p; ++p)
		{
			if ((hex[p - hex] <= 'z') && (hex[p - hex] >= 'a'))
				hex[p - hex] = hex[p - hex] - 32;
			number = number * 16 + (hex[p - hex] >= 'A' ? hex[p - hex] - 'A' + 10 : hex[p - hex] - '0');
		}

		return number;
	}

	void checkspeed()
	{
		unsigned int speed = 0;
		cam0.GetUsbSpeed(speed);
		if (speed == LIBUSB_SPEED_SUPER)
		{
			printf("USB 3.0 device found on cam0!\n");
			cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
		}
		else if (speed == LIBUSB_SPEED_HIGH)
		{
			printf("USB 2.0 device found on cam0!\n");
			cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
		}
		else
		{
			printf("Device speed unknown on cam0!\n");
		}
	}

	cqUsbCamNode() : node_("~")
	{
		image_transport::ImageTransport it(node_);
		pub_img = it.advertise("cam0/image_raw", 1);

		//grab the parameter
		node_.param("exposure", exposure_, -1);
		node_.param("gain", gain_, -1);
		node_.param("trig_mode", trigMode_, std::string("auto"));		// set "external" for using external trigger pin
		node_.param("resolution", resolution_, std::string("752x480")); // can be changed to "640x480"
		node_.param("camera_name", cameraName_, std::string("cam0"));
		node_.param("camera_frame_id", cameraFrameID_, std::string("head_camera"));
		node_.param("frame_rate", frameRate_, int(30));

		//start the camera
		cq_int32_t ret;
		ret = pthread_mutex_init(&mutexDisp, NULL);
		if (ret != 0)
			printf("pthread_mutex_init failed");
		ret = pthread_mutex_init(&mutexCam, NULL);
		if (ret != 0)
			printf("pthread_mutex_init failed");

		cam0.SelectSensor(sensor);

		int usbCnt = CCqUsbCam::OpenUSB();
		printf("%d usb device(s) found!\n", usbCnt);
		if (usbCnt <= 0)
		{
			printf("exiting ...\n");
		}
		cam0.ClaimInterface(0);

		checkspeed();

		pCamInUse = &cam0;

		// set camera parameters
		if (exposure_ >= 0)
		{
			printf("running with exposure value %d\n", exposure_);
			pCamInUse->SetExpoValue(exposure_);
		}
		if (gain_ >= 0)
		{
			printf("running with gain value %d\n", gain_);
			pCamInUse->SetGainValue(gain_);
		}
		if (0 == trigMode_.compare("auto"))
		{
			pCamInUse->SetTrigMode(TRIGMODE_AUTO);
		}
		else if (0 == trigMode_.compare("external"))
		{
			pCamInUse->SetTrigMode(TRIGMODE_SIGNAL);
		}
		else
		{
			printf("bad trigger mode setting");
		}
		if (0 == resolution_.compare("752x480"))
		{
			pCamInUse->SetResolution(RESOLU_752_480);
			if (pCamInUse == &cam0)
			{
				g_width = 752;
				g_height = 480;
			}
		}
		else if (0 == resolution_.compare("640x480"))
		{
			pCamInUse->SetResolution(RESOLU_640_480);
			if (pCamInUse == &cam0)
			{
				g_width = 640;
				g_height = 480;
			}
		}
		else
		{
			printf("bad resolution setting");
		}
	}

	// bool sending_image()
	// {
	// 	cam0.StartCap(g_height, g_width, std::bind(&cqUsbCamNode::Disp, this, std::placeholders::_1));

	// 	signal(SIGALRM, std::bind(&cqUsbCamNode::timerFunction, this, std::placeholders::_1));
	// 	alarm(1);

	// 	printf("Press any key to stop capturing\n");

	// 	getchar();
	// 	pthread_mutex_lock(&mutexCam);
	// 	alarm(0);
	// 	cam0.StopCap();

	// 	pthread_mutex_unlock(&mutexCam);

	// 	pthread_mutex_lock(&mutexDisp);
	// 	cv::destroyWindow("disp");
	// 	cv::waitKey(1);
	// 	cv::waitKey(1);
	// 	cv::waitKey(1);
	// 	cv::waitKey(1);

	// 	pthread_mutex_unlock(&mutexDisp);

	// 	return true;
	// }
};
//}
cqUsbCamNode *camNode;

void Disp(void *frameData)
{
	ros::Rate loopRate(camNode->frameRate_);
	int counter = 1;
	while (ros::ok())
	{

		cv_bridge::CvImage img_bridge;

		pthread_mutex_lock(&camNode->mutexDisp);
		cv::Mat frame(camNode->g_height, camNode->g_width, CV_8UC1, (unsigned char *)frameData);
		//cv::imshow("disp",frame);
		cv::waitKey(1);
		pthread_mutex_unlock(&camNode->mutexDisp);

		std_msgs::Header header; // empty header
		header.seq = counter;	// user defined counter
		counter++;
		header.stamp = ros::Time::now(); // time
		header.frame_id = "cam0";
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame);
		img_bridge.toImageMsg(camNode->img_msg); // from cv_bridge to sensor_msgs::Image
		camNode->pub_img.publish(camNode->img_msg);
		ros::spinOnce();
		loopRate.sleep();
	}
}

void timerFunction(int sig)
{

	unsigned long iByteCntPerSec = 0;
	unsigned long iFrameCntPerSec = 0;

	pthread_mutex_lock(&camNode->mutexCam);

	camNode->cam0.GetRecvByteCnt(iByteCntPerSec);
	camNode->cam0.ClearRecvByteCnt();
	camNode->cam0.GetRecvFrameCnt(iFrameCntPerSec);
	camNode->cam0.ClearRecvFrameCnt();

	printf("cam0: %ld Fps     %0.4f MBs\n", iFrameCntPerSec, float(iByteCntPerSec) / 1024.0 / 1024.0);

	alarm(1);

	pthread_mutex_unlock(&camNode->mutexCam);
}

void sending_image()
{
	camNode->cam0.StartCap(camNode->g_height, camNode->g_width, Disp);

	signal(SIGALRM, timerFunction);
	alarm(1);

	printf("Press any key to stop capturing\n");

	getchar();
	pthread_mutex_lock(&camNode->mutexCam);
	alarm(0);
	camNode->cam0.StopCap();

	pthread_mutex_unlock(&camNode->mutexCam);

	pthread_mutex_lock(&camNode->mutexDisp);
	cv::destroyWindow("disp");
	cv::waitKey(1);
	cv::waitKey(1);
	cv::waitKey(1);
	cv::waitKey(1);

	pthread_mutex_unlock(&camNode->mutexDisp);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cq_camera");
	cqUsbCamNode cn;
	camNode = &cn;

	
	sending_image();
	// if (!sending_image())
	// {
	// 	printf("camera is not responding");
	// }

	return 0;
}
