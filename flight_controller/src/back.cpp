
 Created by kevin on 2020/11/25.


void uavModule::stabilized() {
const double dt = cal.getDt(ros::Time::now().toSec());
updateState(dt);

    desiered.cPosition = Vec3(0,0,1);
    desiered.cPositionD = Vec3(0,0,0);
    desiered.cPositionDD = Vec3(0,0,0);

    Vec3 errorPosition = feedback.cPosition - desiered.cPosition;
    Vec3 errorVelocity = feedback.cPositionD - desiered.cPositionD;
    cout<<errorPosition.transpose()<<endl;
    Eigen::Vector3d F_n = - errorPosition.cwiseProduct(k_bx) - errorVelocity.cwiseProduct(k_bv);
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
    visual.desiredPose.orientation = desiredQuaterniondBodyTF;
    visual.currentPose.orientation = currentQuaterniondBodyTF;



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
    output.force = b3_des.transpose() * F;
    output.rotorRevs = cal.getAllocatedRevs(output.force, moment);
    visual.thrust.data = output.force;
    pubPose();
    for(int i = 0 ; i < 4 ; i ++ ){
//        cout<<output.rotorRevs[i]<<" ";
        wb_motor_set_velocity(motors[i], (output.rotorRevs[i]  * k_f)* pow(-1,(i % 2)));
    }
    cout<<endl;

}
