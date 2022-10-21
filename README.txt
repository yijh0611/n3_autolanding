This is incomplete code

Git library used
1. dji-sdk/Onboard-SDK https://github.com/dji-sdk/Onboard-SDK
2. dji-sdk/Onboard-SDK-ROS https://github.com/dji-sdk/Onboard-SDK-ROS
3. AprilRobotics/apriltag_ros https://github.com/AprilRobotics/apriltag_ros
4. apriltag https://github.com/AprilRobotics/apriltag
5. Some demo ros codes

///////////////////////////////////////////////
rostopic list -v
rostopic echo tf -뒤에 publish되는거 중에 확인하고 싶은거 넣기.
ls -al : 파일 권한 확인하는 명령어 - 파일 권한이 잘못되어 있으면, rosrun으로 인식이 안될 수도 있음.


코드 실행 방법
cm : catkin build -j6가 실행 됨
rl10 : roslaunch dji_sdk check_ros_10이 실행됨
rls1 : roslaunch dji_sdk check_ros_sim_1이 실행됨

단축 명령어 설정
gedit ~/.bashrc
source ~/.bashrc

터미널 창 켜고 끄는 단축키
ctrl + `(숫자 1 옆에있는거)

/////////////////////////
// 이거는 옛날거
터미널에서 roslaunch dji_sdk sdk.launch 실행 - 이거는 드론이 연결되어 있을때만 사용 가능
드론 연결되어 있지 않을때는 roscore 실행

터미널에서 rosrun dji_sdk talker_ros 하면 드론 날리는 코드 실행

/////////////////////////

앞으로(later) 날릴때
1. roslaunch dji_sdk all.launch
 - 이거 실행 다 될때 까지 기다린 후 talker 실행하기
2. rosrun dji_sdk talker_ros_3

3. 로그 기록할 필요 있으면, 노드 하나 더 만들기.

짐벌 확인할 때
1. roslaunch apriltag_ros check_gimbal.launch

/////////////////////////
