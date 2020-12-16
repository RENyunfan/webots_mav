//
// Created by kevin on 2020/11/22.
//

#ifndef FLIGHTCONTROLLER_FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_FLIGHTCONTROLLER_H

#include "flightController/common_inc.h"
#include "flightController/calculate.h"
class uavModule{

private:
    WbDeviceTag camera;
    WbDeviceTag front_left_led;
    WbDeviceTag front_right_led;
    WbDeviceTag imu_uav;
    WbDeviceTag imu_lidar;
    WbDeviceTag gps;
    WbDeviceTag compass;
    WbDeviceTag gyro;
    WbDeviceTag camera_roll_motor;
    WbDeviceTag camera_pitch_motor;
    WbDeviceTag front_left_motor;
    WbDeviceTag front_right_motor;
    WbDeviceTag rear_right_motor;
    WbDeviceTag rear_left_motor;
    WbDeviceTag acc_uav;
    WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
    int time_step;
    int resolution;
    int layers;
    WbDeviceTag sick;
    WbFieldRef robot_translation;
    WbNodeRef uav_node;
    WbNodeRef sick_node;
    const double massQuadrotor = 0.4;
    /// ROS
    geometry_msgs::Pose currentPose;
    ros::Subscriber pose_sub;
    ros::Publisher currentPose_pub;
    ros::Publisher desiredPose_pub;
    ros::Publisher thrust_pub;
    ros::Publisher pointCloud_pub;
    ros::Subscriber velocity_sub;
    ros::NodeHandle nh_;

    Vec3 accErrPositon;
    /// Controller Data Structure
    cal_c cal;
    struct desiered_t {
        Vec3 cPosition;
        Mat33 cRotiation;

        Vec3 pPosition;
        Mat33 pRotiation;

        Vec3 ppPosition;
        Mat33 ppRotiation;

        Vec3 cPositionD;
        Vec3 cPositionDD;
        Vec3 cEulerAngle;

        Vec3 cEulerAngleD;
        Vec3 pEulerAngleD;
        Vec3 ppEulerAngleD;

        Mat33 rotation;
        Vec3 bodyRate;
        double thrust;
    }desiered;

    struct feedback_t {
        Vec3 cPosition;
        Mat33 cRotiation;

        Vec3 pPosition;
        Vec3 pPositionD;
        Mat33 pRotiation;

        Vec3 ppPosition;
        Vec3 ppPositionD;
        Mat33 ppRotiation;

        Vec3 cPositionD;
        Vec3 cPositionDD;

        Vec3 cEulerAngle;

        Vec3 cEulerAngleD;
        Vec3 pEulerAngleD;
        Vec3 ppEulerAngleD;
    }feedback;

    struct calculate_t {
        Vec3 cPosition;
        Mat33 cRotiation;

        Vec3 pPosition;
        Mat33 pRotiation;

        Vec3 ppPosition;
        Mat33 ppRotiation;

        Vec3 cPositionD;
        Vec3 cPositionDD;
    }calculate;

    struct output_t{
        double force;
        Vec4 rotorRevs;
        Vec3 eulerAngle;
        Mat33 rotationMatrix;
        geometry_msgs::Quaternion quaternion;

    }output;

    struct visual_t{
        geometry_msgs::PoseStamped currentPose;
        geometry_msgs::PoseStamped desiredPose;
        std_msgs::Float32 thrust;
    }visual;

    double time;

    Vec3 k_bx,k_bv,k_p,k_d,k_bi ;
    double k_f;
    Mat33 J;
    ros::Time now;


    void cmd_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void cmd_attitude_callback(const mavros_msgs::AttitudeTargetConstPtr &msg);
    void cmd_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    Vec3 lockPosition;
    int isLock;

public:
    uavModule(int time_step, Eigen::MatrixXd paraMat);
    void runSickOnce();
    void runUavControllerOnce();
    void getPose();
    geometry_msgs::Pose getPoseOnce();
    void pubPose();
    void stabilized();
    void updateState(double dt);
    void setDesiredPosition(Vec3 position);
    void velocityControl();
};

#endif //FLIGHTCONTROLLER_FLIGHTCONTROLLER_H
