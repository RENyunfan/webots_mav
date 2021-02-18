//
// Created by kevin on 2020/11/22.
//


#include "flightController/common_inc.h"
#include "flightController/flightController.h"
#define SIGN(x) ((x) > 0) - ((x) < 0)
std::string config_file = "../../config/default.yaml";

int main(int argc, char **argv) {
    cout<<"Start drone init process."<<endl;
    wb_robot_init();
    ros::init(argc,argv,"uav_lidar");
    ros::NodeHandle nh;
    // 读取PID参数
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    cv::Mat paraMatCV, targetMatCV;
    fs["parameterMatrix"] >> paraMatCV;
    fs["targetMatrix"] >> targetMatCV;
    Eigen::MatrixXd  paraMat(6,3);
    Eigen::MatrixXd  targetMat(3,3);
    cv::cv2eigen(paraMatCV,paraMat);
    cv::cv2eigen(targetMatCV,targetMat);
    std::cout<<paraMat<<std::endl;
    ros::Rate loopRate(100);

    const int time_step = 10;
    uavModule UAV(nh,time_step, paraMat);
    int cnt = 100;
    while (wb_robot_step(time_step) != -1) {
    if(cnt -- < 0 ){
        ROS_INFO("Position control working!");
        cnt = 100;
    }
//        UAV.setDesiredPosition(targetMat.row(0));
//        UAV.velocityControl();
//        UAV.stabilized();
        UAV.positionControl();
        ros::spinOnce();
//        loopRate.sleep();
    }

    wb_robot_cleanup();

    return EXIT_SUCCESS;
}
