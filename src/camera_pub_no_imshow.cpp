#include <iostream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"
// fps 30으로 줄였음

using namespace cv;
using namespace std;

sensor_msgs::CameraInfo cam_par(){ 
	sensor_msgs::CameraInfo cam_info;

	int width = 640;
	int height = 480;

	float fx = 501.92767418;
	float fy = 500.12588919;
	float cx = 331.22594614;
	float cy = 225.79436303;

	cam_info.width = width;
	cam_info.height = height;

	cam_info.K = {fx,0,cx,0,fy,cy,0,0,1}; //{{fx,0,cx},{0,fy,cy},{0,0,1}};
	cam_info.D = {0,0,0,0};
	cam_info.P = {fx,0,cx,0,0,fy,cy,0,0,0,1,0}; // {{fx,0,cx,0},{0,fy,cy,0},{0,0,1,0}};

	return cam_info;
}

class CvImage
{
  sensor_msgs::ImagePtr toImageMsg() const;

  // Overload mainly intended for aggregate messages that contain
  // a sensor_msgs::Image as a member.
  void toImageMsg(sensor_msgs::Image& ros_image) const;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_pub");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera_rect/image_rect", 1);

    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_rect/camera_info", 10);

    // OpenCV 관련
	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 640); // 1280
	cap.set(CAP_PROP_FRAME_HEIGHT, 480); // 720
	// cap.set(CAP_PROP_FPS, 50);
	double fps = cap.get(CAP_PROP_FPS);
	cout << fps << "fps" << endl;
	// cout << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	
	if (!cap.isOpened())
	{
		cout << "Can't open the camera" << endl;
		return -1;
	}

	Mat img;
	// Mat img_tmp;
	// bool is_first = true;

	while (1)
	{
		cap >> img;

		// imshow("camera img", img); // 나중에 코드 실행할 때는 이거 없애면 좀 더 빨라질 듯
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		pub.publish(msg); // 이미지 출력
		sensor_msgs::CameraInfo cam_info = cam_par();
		// ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_rect/camera_info", 10); // camera_rect/camera_info
		cam_info_pub.publish(cam_info);

		// if (waitKey(1) == 27)
		// 	break;
	}

	return 0;
}


