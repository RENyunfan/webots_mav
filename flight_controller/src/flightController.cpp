//
// Created by kevin on 2020/11/22.
//

#include "flightController/flightController.h"

void uavModule::cmd_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg){

    desiered.yawD = msg->twist.angular.z;

    desiered.cPositionD.x() = 3*(msg->twist.linear.x * cos(feedback.cEulerAngle.z())  + msg->twist.linear.y*sin(feedback.cEulerAngle.z()));
    desiered.cPositionD.y() = 3*(msg->twist.linear.x * sin(feedback.cEulerAngle.z()) - msg->twist.linear.y*cos(feedback.cEulerAngle.z()));
    desiered.cPositionD.z() = msg->twist.linear.z;
    desiered.cEulerAngleD.z() = msg->twist.angular.z;
    desiered.cPosition.z()=-1;

}

void uavModule::cmd_traj_callback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
    desiered.cPosition = Vec3(msg->position.x, -msg->position.y, msg->position.z);
    desiered.cPositionD = Vec3(msg->velocity.x, -msg->velocity.y, msg->velocity.z);
    desiered.cPositionDD = Vec3(msg->acceleration.x, -msg->acceleration.y, msg->acceleration.z);
    desiered.yaw = -msg->yaw;
    desiered.yawD =-msg->yaw_dot;
}


void uavModule::cmd_attitude_callback(const mavros_msgs::AttitudeTargetConstPtr &msg) {
     desiered.thrust = msg->thrust;
     desiered.bodyRate = uav_utils::vec3_tf2eig(msg->body_rate);
     desiered.rotation = uav_utils::quad_tf2eig(msg->orientation).toRotationMatrix();
}

void uavModule::cmd_position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    desiered.cPosition = Vec3(msg->pose.position.x,-msg->pose.position.y, msg->pose.position.z);
    desiered.cRotiation = uav_utils::quad_tf2eig(msg->pose.orientation).toRotationMatrix();
    desiered.cPositionD.setZero();
    desiered.cPositionDD.setZero();
    desiered.yaw = 0;
    desiered.yawD = 0;
}


uavModule::uavModule(ros::NodeHandle & nh,int time_, Eigen::MatrixXd paraMat) {
    nh_ = nh;
    /// For visulization
    currentPose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/pose/current", 1);
    desiredPose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/pose/desired", 1);
    thrust_pub = nh_.advertise<std_msgs::Float32>("/pose/thrust",1);
    pointCloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/sensing/point_cloud/lidar",20000);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom",100);
    traj_sub = nh_.subscribe("/planning/pos_cmd",1,&uavModule::cmd_traj_callback, this);
    velocity_sub = nh_.subscribe("/mavros/setpoint_velocity/cmd_vel", 1, &uavModule::cmd_velocity_callback, this);
    pose_sub = nh_.subscribe("/mavros/setpoint_position/local",1,&uavModule::cmd_position_callback, this);
    k_bx = paraMat.row(0);
    k_bv = paraMat.row(1);
    k_bi = paraMat.row(2);
    k_p = paraMat.row(3);
    k_d = paraMat.row(4);
    k_f = paraMat.row(5)(0);

    J << 0.000913, 0, -0.00035,
        0,   0.002364 , 0,
        -0.0003579, 0, 0.00279965;
    time_step = time_;

    desiered.cPosition = Vec3(0,0,2);
    desiered.cPositionD = Vec3(0,0,0);
    desiered.cPositionDD = Vec3(0,0,0);
    desiered.yaw = 0;
    desiered.yawD = 0;

    /// Get UAV handle
    uav_node = wb_supervisor_node_get_from_def("UAV_LIDAR");
    sick_node = wb_supervisor_node_get_from_def("LIDAR");
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, time_step);
    front_left_led = wb_robot_get_device("front left led");
    front_right_led = wb_robot_get_device("front right led");
    imu_uav = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu_uav, time_step);
    acc_uav = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(acc_uav, time_step);
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, time_step);
    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, time_step);
    gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, time_step);
    wb_keyboard_enable(time_step);
    camera_roll_motor = wb_robot_get_device("camera roll");
    camera_pitch_motor = wb_robot_get_device("camera pitch");
    // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.
    // Get propeller motors and set them to velocity mode.
    front_left_motor = wb_robot_get_device("front left propeller");
    front_right_motor = wb_robot_get_device("front right propeller");
    rear_left_motor = wb_robot_get_device("rear left propeller");
    rear_right_motor = wb_robot_get_device("rear right propeller");
    motors[0] = front_left_motor;
    motors[1] = front_right_motor;
    motors[2] = rear_right_motor;
    motors[3] = rear_left_motor;
    int m;
    for (m = 0; m < 4; ++m) {
        wb_motor_set_position(motors[m], INFINITY);
        wb_motor_set_velocity(motors[m], 1.0);
    }
    // Display the welcome message.
    printf("Start the drone...\n");
    // Wait one second.
    while (wb_robot_step(time_step) != -1) {
        if (wb_robot_get_time() > 1.0)
            break;
    }
    /*  Enable Sick */

    sick = wb_robot_get_device("sick");
    wb_lidar_enable(sick, time_step);
    wb_lidar_enable_point_cloud(sick);
    resolution = wb_lidar_get_horizontal_resolution(sick);
    layers = wb_lidar_get_number_of_layers(sick);

    /* Init SO3 Controller */
    controller_.setMass(massQuadrotor);
    controller_.setGravity(cal.g);
}

//TODO: I have limit the point cloud range to [0,4] meter. To make sure the color map in rviz looks good
void uavModule::runSickOnce() {
    sensor_msgs::PointCloud2 pc;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    const int size = wb_lidar_get_number_of_points(sick);// * sizeof(WbLidarPoint);
    cloud.width = size;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    int cnt = 0;
    for (int i = 0 ; i < layers ; i++){
        const WbLidarPoint *layer = wb_lidar_get_layer_point_cloud(sick,i);
        int p;
        for (p = 0; p < resolution; ++p) {
            WbLidarPoint point = layer[p];
//            if( -point.y < 4 && -point.y > 0-1e-3)
            {
                cloud.points[cnt].x = -point.z;
                cloud.points[cnt].y = point.x;
                cloud.points[cnt].z = -point.y/1.265f;
                cnt++;
            }
        }
    }
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "laser";
    output.header.stamp = ros::Time::now();
    pointCloud_pub.publish(output);
}

void uavModule::updateState(double dt) {
    /*
     * Update the uav state
     *  feedback: cPosition, cPositionD, cPositionDD
     *            cRotiation, cEulerAngle, cEulerAngleD
     *
     * */
    const double roll = (wb_inertial_unit_get_roll_pitch_yaw(imu_uav)[0] + M_PI / 2);
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu_uav)[1];
    const double yaw = -wb_inertial_unit_get_roll_pitch_yaw(imu_uav)[2];
    const double * position =  wb_supervisor_node_get_position(uav_node);
    const double * orientation =  wb_supervisor_node_get_orientation(uav_node);
    const double * positionD = wb_supervisor_node_get_velocity(uav_node);
    const double * angulerD = wb_gyro_get_values(gyro);
    const double * positionDD = wb_accelerometer_get_values(acc_uav);


    controller_.setPosition( Vec3(position[0],position[2],position[1]));
    controller_.setVelocity(Vec3( positionD[0],positionD[2],positionD[1]));
    controller_.setAcc(Vec3( positionDD[0],positionDD[2],positionDD[1]));

    feedback.cPosition << position[0],position[2],position[1];
    feedback.cPositionD << positionD[0],positionD[2],positionD[1];
    feedback.cPositionDD =Vec3( positionDD[0],positionDD[2],positionDD[1]);
    feedback.cEulerAngleD = Vec3(angulerD[0],angulerD[1],angulerD[2]);
    feedback.cEulerAngle = Vec3(roll,pitch,yaw);
    feedback.cRotiation = cal.eulerAngle2rotation(feedback.cEulerAngle);


}

void uavModule::setDesiredPosition(Vec3 position){
    desiered.cPosition = position;
}

void uavModule::positionControl(){
    now = ros::Time::now();
    const double dt = cal.getDt(ros::Time::now().toSec());
    desiered.yaw-= desiered.yawD * dt;
    updateState(dt);

    controller_.calculateControl(desiered.cPosition, desiered.cPositionD, desiered.cPositionDD, desiered.yaw,desiered.yawD,k_bx,k_bv);
    Vec3 force_ = controller_.getComputedForce();
    Vec3 b3c;
    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    Eigen::Matrix3d desiredRotationMatrix = controller_.getComputedOrientation().toRotationMatrix();
    Vec3 desieredEulerAngle;
    desieredEulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);

    geometry_msgs::Quaternion desiredQuaterniondBodyTF;
    geometry_msgs::Quaternion currentQuaterniondBodyTF;

    desiredQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(desieredEulerAngle[0],desieredEulerAngle[1],desieredEulerAngle[2]);
    currentQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(feedback.cEulerAngle[0],-feedback.cEulerAngle[1],-feedback.cEulerAngle[2]);
    visual.desiredPose.pose.orientation = desiredQuaterniondBodyTF;
    visual.currentPose.pose.orientation = currentQuaterniondBodyTF;
    visual.desiredPose.pose.position.x = desiered.cPosition.x();visual.desiredPose.pose.position.y = desiered.cPosition.y();visual.desiredPose.pose.position.z = desiered.cPosition.z();
    visual.currentPose.pose.position.x = feedback.cPosition.x();visual.currentPose.pose.position.y = feedback.cPosition.y();visual.currentPose.pose.position.z = feedback.cPosition.z();

    Vec3 errorRotation =
            0.5 * cal.antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * feedback.cRotiation -
                                                   feedback.cRotiation.transpose() * desiredRotationMatrix));

    Vec3 errorAngular = feedback.cEulerAngleD; //- feedback.cRotiation.transpose() * desiredRotationMatrix * desieredEulerAngle;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

//    cout<<errorRotation.transpose()<<endl;
    Vec3 moment =
            errorRotation.cwiseProduct(k_p) + errorAngular.cwiseProduct(k_d)
            + feedback.cEulerAngleD.cross(J * feedback.cEulerAngleD);

//            保方向饱和函数
//    if (moment.norm() > 1)
//    {
//        moment = moment.normalized();
//    }
    output.force = b3c.transpose() * force_;
    output.rotorRevs = cal.getAllocatedRevs(output.force, moment);
    visual.thrust.data = output.force;
    pubPose();
    for(int i = 0 ; i < 4 ; i ++ ){
//        cout<<(output.rotorRevs[i]  * k_f)* pow(-1,(i % 2))<<" ";
        wb_motor_set_velocity(motors[i], (output.rotorRevs[i]  * k_f)* pow(-1,(i % 2)));
    }
//    cout<<endl;

}

void uavModule::stabilized() {
    now = ros::Time::now();
    const double dt = cal.getDt(ros::Time::now().toSec());
    updateState(dt);
    desiered.cPositionD = Vec3(0,0,0);
    desiered.cPositionDD = Vec3(0,0,0);

    Vec3 errorPosition = feedback.cPosition - desiered.cPosition;
    Vec3 errorVelocity = feedback.cPositionD - desiered.cPositionD;
    Vec3 errorAcc = feedback.cPositionDD - desiered.cPositionDD;

    Eigen::Vector3d totalError =
            -errorPosition -errorVelocity - errorAcc;

    Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                       fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                       fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

    Vec3 force_;
    force_.noalias() =
            k_bx.asDiagonal() * (-errorPosition) + k_bv.asDiagonal() * (-errorVelocity) +
            massQuadrotor * /*(Eigen::Vector3d(1, 1, 1) - ka).asDiagonal() **/ (desiered.cPositionDD) +
            massQuadrotor * ka.asDiagonal() * (-errorAcc) +
            massQuadrotor * cal.g * Eigen::Vector3d(0, 0, 1);


    // Limit control angle to 45 degree
    double          theta = M_PI / 2;
    double          c     = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = k_bx.asDiagonal() * (-errorPosition) +
                  k_bv.asDiagonal() * (-errorVelocity) + //
                  massQuadrotor * desiered.cPositionDD +                    //
                  massQuadrotor * ka.asDiagonal() * (-errorAcc);
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c/2)
    {
        double nf        = f.norm();
        double A         = c * c * nf * nf - f(2) * f(2);
        double B         = 2 * (c * c - 1) * f(2) * massQuadrotor * cal.g;
        double C         = (c * c - 1) * massQuadrotor * massQuadrotor * cal.g * cal.g;
        double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + massQuadrotor * cal.g * Eigen::Vector3d(0, 0, 1);
    }


    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(0), sin(0), 0);

    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d desiredRotationMatrix;
    desiredRotationMatrix << b1c, b2c, b3c;

    Vec3 eulerAngle;
    eulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);

    Vec3 desieredEulerAngle;
    desieredEulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);

    geometry_msgs::Quaternion desiredQuaterniondBodyTF;
    geometry_msgs::Quaternion currentQuaterniondBodyTF;

    desiredQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(desieredEulerAngle[0],desieredEulerAngle[1],desieredEulerAngle[2]);
    currentQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(feedback.cEulerAngle[0],-feedback.cEulerAngle[1],-feedback.cEulerAngle[2]);
    visual.desiredPose.pose.orientation = desiredQuaterniondBodyTF;
    visual.currentPose.pose.orientation = currentQuaterniondBodyTF;



    Vec3 errorRotation =
            0.5 * cal.antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * feedback.cRotiation -
                                                   feedback.cRotiation.transpose() * desiredRotationMatrix));

    Vec3 errorAngular = feedback.cEulerAngleD; //- feedback.cRotiation.transpose() * desiredRotationMatrix * desieredEulerAngle;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

//    cout<<errorRotation.transpose()<<endl;
    Vec3 moment =
            errorRotation.cwiseProduct(k_p) + errorAngular.cwiseProduct(k_d)
            + feedback.cEulerAngleD.cross(J * feedback.cEulerAngleD);

//            保方向饱和函数
    if (moment.norm() > 1)
    {
        moment = moment.normalized();
    }
    output.force = b3c.transpose() * force_;
    output.rotorRevs = cal.getAllocatedRevs(output.force, moment);
    visual.thrust.data = output.force;
    pubPose();
    for(int i = 0 ; i < 4 ; i ++ ){
//        cout<<output.rotorRevs[i]<<" ";
        wb_motor_set_velocity(motors[i], (output.rotorRevs[i]  * k_f)* pow(-1,(i % 2)));
    }
//    cout<<endl;

}

void uavModule::pubPose() {
    tf::Quaternion quad_uav;
    tf::Transform trans;

    static tf::TransformBroadcaster broadcaster;
//    tf::Quaternion quad;
//    tf::Transform trans;
//
    trans.setOrigin(tf::Vector3(feedback.cPosition.x(), -feedback.cPosition.y(), feedback.cPosition.z()));
    quad_uav.setRPY(feedback.cEulerAngle[0],-feedback.cEulerAngle[1],-feedback.cEulerAngle[2]);
    trans.setRotation(quad_uav);
    broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(),"world","uav"));

    const double * position =  wb_supervisor_node_get_position(sick_node);
//    const double * R = wb_supervisor_node_get_orientation(sick_node);
//    Mat33 rotation;
//    rotation<<  R[0],R[1],R[2],
//            R[3],R[4],R[5],
//            R[6],R[7],R[8];
//    Mat33 rot90;
//    rot90 << cos(M_PI / 2), sin(M_PI / 2), 0,
//            -sin(M_PI / 2), cos(M_PI / 2), 0 ,
//            0 , 0 , 1;
//    Eigen::Quaterniond quad_eig = cal.rotation2quatern(rotation * rot90);
//    tf::Quaternion quad(quad_eig.x(), quad_eig.y(), quad_eig.z(), -quad_eig.w());
    tf::Vector3 posi(position[0],-position[2],position[1]);

    trans.setOrigin(posi);
    trans.setRotation(quad_uav);
    broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(),"world","laser"));
//    trans.setOrigin(tf::Vector3(0,0,0.08));
//    trans.setRotation( tf::Quaternion(-0.707107, 0, 0, 0.707107));
//    broadcaster.sendTransform(tf::StampedTransform(trans, ros::Time::now(),"uav","laser"));

    runSickOnce();

    visual.currentPose.pose.position.x = feedback.cPosition.x();
    visual.currentPose.pose.position.y = -feedback.cPosition.y();
    visual.currentPose.pose.position.z = feedback.cPosition.z();
    visual.currentPose.header.frame_id = "world";
    visual.currentPose.header.stamp = ros::Time::now();
    currentPose_pub.publish(visual.currentPose);
    desiredPose_pub.publish(visual.desiredPose);
    thrust_pub.publish(visual.thrust);

    nav_msgs::Odometry odom_;
    odom_.pose.pose = visual.currentPose.pose;
    odom_.twist.twist.linear.x = feedback.cPositionD.x();
    odom_.twist.twist.linear.y = -feedback.cPositionD.y();
    odom_.twist.twist.linear.z = feedback.cPositionD.z();
    odom_.twist.twist.angular.x = feedback.cEulerAngleD.x();
    odom_.twist.twist.angular.y = feedback.cEulerAngleD.y();
    odom_.twist.twist.angular.z = feedback.cEulerAngleD.z();
    odom_.header.stamp = ros::Time::now();
    odom_.header.frame_id = "world";
    odom_pub.publish(odom_);

}

void uavModule::velocityControl(){
    const double dt = cal.getDt(ros::Time::now().toSec());
    updateState(dt);

    if(desiered.cPositionD.norm() <0.01){
        if(!isLock){
            lockPosition = feedback.cPosition;
            isLock = 1;
        }
        setDesiredPosition(lockPosition);
        stabilized();
        return;
    }
    else
        isLock = 0;


    Vec3 errorVelocity = feedback.cPositionD - desiered.cPositionD;
//    cout<<errorVelocity.transpose()<<endl;
    Eigen::Vector3d F_n = - errorVelocity.cwiseProduct(k_bv) - massQuadrotor * desiered.cPositionDD ;
    F_n += (massQuadrotor) * (cal.e3 * cal.g);
    Eigen::Vector3d F = F_n;

    //保方向饱和函数
    if (F.norm() > 5)
    {
        F = 5 * F / F.norm() ;
    }

    //2,1,0->ZYX
    Eigen::Vector3d eulerAngle(0,0,0);
    Eigen::Vector3d b1_des(cos(eulerAngle.x()), sin(eulerAngle.x()), 0);

    Eigen::Vector3d b3_des = F / F.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des /= b2_des.norm();

    Eigen::Matrix3d desiredRotationMatrix;
    desiredRotationMatrix.col(0) = b2_des.cross(b3_des);
    desiredRotationMatrix.col(1) = b2_des;
    desiredRotationMatrix.col(2) = b3_des;

    eulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);

    Vec3 desieredEulerAngle;

    desieredEulerAngle = desiredRotationMatrix.eulerAngles(0,1,2);
    geometry_msgs::Quaternion desiredQuaterniondBodyTF;
    geometry_msgs::Quaternion currentQuaterniondBodyTF;

    desiredQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(desieredEulerAngle[0],desieredEulerAngle[1],desieredEulerAngle[2]);
    currentQuaterniondBodyTF = tf::createQuaternionMsgFromRollPitchYaw(feedback.cEulerAngle[0],feedback.cEulerAngle[1],feedback.cEulerAngle[2]);
    visual.desiredPose.pose.orientation = desiredQuaterniondBodyTF;
    visual.currentPose.pose.orientation = currentQuaterniondBodyTF;


    Vec3 errorRotation =
            0.5 * cal.antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * feedback.cRotiation -
                                                   feedback.cRotiation.transpose() * desiredRotationMatrix));

    Vec3 errorAngular = feedback.cEulerAngleD ; //- feedback.cRotiation.transpose() * desiredRotationMatrix * desieredEulerAngle;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

//    cout<<errorRotation.transpose()<<endl;
    Vec3 moment =
            errorRotation.cwiseProduct(k_p) + errorAngular.cwiseProduct(k_d)
            + feedback.cEulerAngleD.cross(J * feedback.cEulerAngleD);

//            保方向饱和函数
    if (moment.norm() > 1)
    {
        moment = moment.normalized();
    }
    output.force = b3_des.transpose() * F;
    output.rotorRevs = cal.getAllocatedRevs(output.force, moment);
    visual.thrust.data = output.force;
    pubPose();
    for(int i = 0 ; i < 4 ; i ++ ){
//        cout<<output.rotorRevs[i]<<" ";
        wb_motor_set_velocity(motors[i], (output.rotorRevs[i]  * k_f)* pow(-1,(i % 2)));
    }
//    cout<<endl;



}

