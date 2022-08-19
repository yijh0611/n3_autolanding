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

using namespace cv;
using namespace std;

// def make_camera_msg(cam): // python
//     camera_info_msg = CameraInfo()
//     width, height = cam[0], cam[1]
//     fx, fy = cam[2], cam[3]
//     cx, cy = cam[4], cam[5]
//     camera_info_msg.width = width
//     camera_info_msg.height = height
//     camera_info_msg.K = [fx, 0, cx,
//                          0, fy, cy,
//                          0, 0, 1]
                         
//     camera_info_msg.D = [0, 0, 0, 0]
    
//     camera_info_msg.P = [fx, 0, cx, 0,
//                          0, fy, cy, 0,
//                          0, 0, 1, 0]
//     return camera_info_msg

// def make_camera_msg2(): // python
//     camera_info_msg = CameraInfo()
//     width, height = 640, 480
//     fx, fy = 1.638612, 2.963136
//     cx, cy = 320, 240
//     camera_info_msg.width = width
//     camera_info_msg.height = height
//     camera_info_msg.K = [fx, 0, cx,
//                          0, fy, cy,
//                          0, 0, 1]
                         
//     camera_info_msg.D = [0, 0, 0, 0]
    
//     camera_info_msg.P = [fx, 0, cx, 0,
//                          0, fy, cy, 0,
//                          0, 0, 1, 0]
//     return camera_info_msg

sensor_msgs::CameraInfo cam_par(){ //카메라 정보 구해서 추가하면 됨. 위에 파이썬 코드 보고 참고하면 될 듯
	sensor_msgs::CameraInfo cam_info;
	int width = 640;
	int height = 480;

	float fx = 501.92767418;
	float fy = 500.12588919;
	float cx = 331.22594614;
	float cy = 225.79436303;

	cam_info.width = width;
	cam_info.width = height;

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

    // ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_rect/camera_info", 10);

    // OpenCV 관련
	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FPS, 120);
	double fps = cap.get(CAP_PROP_FPS);
	cout << fps << "fps" << endl;
	// cout << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	
	if (!cap.isOpened())
	{
		cout << "Can't open the camera" << endl;
		return -1;
	}

	Mat img;

	while (1)
	{
		cap >> img;

		imshow("camera img", img); // 나중에 코드 실행할 때는 이거 없애면 좀 더 빨라질 듯
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg); // 이미지 출력
		sensor_msgs::CameraInfo cam_info = cam_par();
		ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_rect/camera_info", 10); // camera_rect/camera_info
		cam_info_pub.publish(cam_info);

		if (waitKey(1) == 27)
			break;
	}

	return 0;
}
