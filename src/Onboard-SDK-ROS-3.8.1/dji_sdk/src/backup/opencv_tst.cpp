
// // // OpenCV 를 사용하기 위해서 3개의 헤더파일을 include 합니다.
// // #include <opencv2/core.hpp>
// // #include <opencv2/imgcodecs.hpp>
// // #include <opencv2/highgui.hpp>

// // #include <opencv2/imgproc/imgproc.hpp>

// // // cout 을 사용하기 위해서 <iostream> 헤더파일을 include 합니다. 
// // // <iostream> 은 C++ 에서 입출력을 위한 헤더파일입니다. 
// // // 이는, C++ 표준 라이브러리의 하나이며, C 언어에서의 <stdio.h> 와 같은 역할을 합니다.
// // // using namespace std; 를 작성합니다.
// // #include <iostream>

// // // OpenCV 의 모든 함수와 클래스는 cv namespace 안에 있습니다.
// // // 모든 라인에 cv:: 을 쓰는 것을 막기 위해 namespace 를 작성합니다.
// // using namespace cv;

// // // #include <iostream>
// // using namespace std;

// // // argc ( Argument Count )는 argument의 수를 의미합니다. 
// // // argv ( Argument Vector )는 argument 가 char 형으로 저장이 되는 변수입니다.
// // // argv[0] 은 실행 파일 경로가 저장됩니다.
// // // argv[1], argv[2] … 에는 순서대로 사용자가 입력한 argument 가 저장 됩니다.
// // // int main( int argc, char** argv )
// // // {
// // //     // Mat 객체를 생성하여 로드된 영상 데이터를 저장합니다.
// // //     Mat image;

// // //     // imread() : 영상을 읽어옵니다. 
// // //     // IMREAD_COLOR라는 형식으로 파일명.파일확장자 영상을 읽어옵니다.
// // //     image = imread( "/home/aims/AprilTag_08.png", IMREAD_COLOR );

// // //     // 유효하지 않은 입력 영상에 대한 확인 과정입니다. 
// // //     if( image.empty() )
// // //     {
// // //         cout << "이미지 파일을 찾을 수 없습니다." << std::endl ;
// // //         return -1;
// // //     }

// // //     // namedWindow() : 영상을 화면에 보이게 하기 위해서 창을 생성합니다.
// // //     // WINDOW_AUTOSIZE : 이미지의 크기에 따라 창의 크기가 결정됩니다.
// // //     namedWindow( "창을 생성하였습니다.", WINDOW_AUTOSIZE );

// // //     // imshow() : OpenCV 윈도우에 새로운 이미지를 업데이트 해줍니다.
// // //     imshow( "OpenCV 창에 새로운 영상을 업데이트 하였습니다.", image );

// // //     // waitKey : 사용자의 키보드 입력을 기다리는 함수 입니다.
// // //     // 0 : 무한대를 의미합니다.
// // //     // 사용자가 키보드를 입력하기 전까지 윈도우 창은 닫히지 않습니다.
// // //     waitKey(0);

// // //     return 0;
// // // }

// // int main(){
// //     cout << 1 << endl;
// // }



// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include <sstream>

// #include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <iostream>

// #include <opencv2/videoio.hpp>
// #include <stdio.h>

// #include "opencv2/opencv.hpp"

// using namespace cv;
// using namespace std;


// // int main(int argc, char** argv) {

// // 	ros::init(argc, argv, "opencv_version_test");
// // 	ros::NodeHandle nh;
// // 	cout << "OpenCV version : " << CV_VERSION << endl;
 
// // 	return 0;
// // }

// int main(int, char**)
// {
//     Mat src;
//     // use default camera as video source
//     VideoCapture cap(0);
//     // // check if we succeeded
//     if (!cap.isOpened()) {
//         cerr << "ERROR! Unable to open camera\n";
//         return -1;
//     }
//     // get one frame from camera to know frame size and type
//     cap >> src;
//     // check if we succeeded
//     if (src.empty()) {
//         cerr << "ERROR! blank frame grabbed\n";
//         return -1;
//     }
//     bool isColor = (src.type() == CV_8UC3);
//     //--- INITIALIZE VIDEOWRITER
//     VideoWriter writer;
//     int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
//     double fps = 25.0;                          // framerate of the created video stream
//     string filename = "./live.avi";             // name of the output video file
//     writer.open(filename, codec, fps, src.size(), isColor);
//     // check if we succeeded
//     if (!writer.isOpened()) {
//         cerr << "Could not open the output video file for write\n";
//         return -1;
//     }
//     //--- GRAB AND WRITE LOOP
//     cout << "Writing videofile: " << filename << endl
//          << "Press any key to terminate" << endl;
//     for (;;)
//     {
//         // check if we succeeded
//         if (!cap.read(src)) {
//             cerr << "ERROR! blank frame grabbed\n";
//             break;
//         }
//         // encode the frame into the videofile stream
//         writer.write(src);
//         // show live and wait for a key with timeout long enough to show images
//         imshow("Live", src);
//         if (waitKey(5) >= 0)
//             break;
//     }
//     // // the videofile will be closed and released automatically in VideoWriter destructor
//     return 0;
// }



// // int main(int argc, char **argv)
// // {
// //     ros::init(argc, argv, "opencv_tst");
// //     ros::NodeHandle n;
// //     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// //     ros::Rate loop_rate(10);
// //     while (ros::ok()){
// //         std_msgs::String msg;
// //         stringstream ss;
// //         ss << "hello world ";
// //         msg.data = ss.str();
// //         ROS_INFO("%s", msg.data.c_str());
// //         chatter_pub.publish(msg);
// //         ros::spinOnce();
// //         loop_rate.sleep();
// //         cout << "OpenCV Version: "<< CV_VERSION << endl;
// //         //Video (note book cam)
// //         // VideoCapture cap(0); // access First connected webcam
// //         // if(!cap.isOpened())
// //         // {
// //         //     cout << "Capture(Webcam) Couldn't open." << endl;
// //         //     return -1;
// //         // }
    
// //         // namedWindow("Video");
        
// //         // Mat frame;
        
// //         // while(char(waitKey(1)) != 'q' && cap.isOpened())
// //         // {
// //         //     cap >> frame; // get a frame from captures
// //         //     if(frame.empty())
// //         //     {
// //         //         cout << "Video over!" << endl;
// //         //         break;
// //         //     }
// //         //     //Show frame
// //         //     imshow("Video", frame);
// //         // }
// //     }
// //     return 0;
// // }


#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <ctime>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int ac, char** av) {

	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(CAP_PROP_FRAME_HEIGHT, 720);
	cap.set(CAP_PROP_FPS, 120);
	double fps = cap.get(CAP_PROP_FPS);
	cout << fps << "fps" << endl;
	// int width = 
	cout << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
	
	if (!cap.isOpened())
	{
		printf("Can't open the camera");
		return -1;
	}

	Mat img;

	// For counting frames per sec
	// clock_t start = clock();
	ros::Time::init();
	ros::Time start_time = ros::Time::now();
	int i = 1;
	// while(1)
	// {
	// 	printf("%0.5f\n",(float)(clock() - start)/CLOCKS_PER_SEC);
	// }

	while (1)
	{
		cap >> img;

		imshow("camera img", img);
		i = i + 1;

		// ros::Time finish_time = ros::Time::now();
		// float duration = (float)finish_time - (float)start_time;
		// cout << duration << endl;

		if(ros::Time::now() - start_time > ros::Duration(1)){
			start_time = ros::Time::now();
			cout << i << "fps" << endl;
			i = 0;
		}
		
		// cout << finish_time - start_time << endl;
		// printf("%0.5f\n",(float)(clock() - start)/CLOCKS_PER_SEC);
		// cout << i << endl;

		if (waitKey(1) == 27)
			break;
	}

	return 0;
}