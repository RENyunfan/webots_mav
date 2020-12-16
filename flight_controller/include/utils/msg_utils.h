//
// Created by kevin on 2020/11/25.
//

#ifndef FLIGHTCONTROLLER_MSG_UTILS_H
#define FLIGHTCONTROLLER_MSG_UTILS_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
namespace uav_utils{


    template <typename Scalar_t = double>
    Eigen::Matrix<Scalar_t, 3, 1> vec3_tf2eig(const geometry_msgs::Vector3& msg) {
        return Eigen::Matrix<Scalar_t, 3, 1>(msg.x, msg.y, msg.z);
    }

    template <typename Derived>
    geometry_msgs::Vector3 vec3_eig2tf(const Eigen::DenseBase<Derived>& v) {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

        geometry_msgs::Vector3 msg;
        msg.x = v.x();
        msg.y = v.y();
        msg.z = v.z();
        return msg;
    }

    template <typename Scalar_t = double>
    Eigen::Matrix<Scalar_t, 3, 1> point_tf2eig(const geometry_msgs::Point& msg) {
        return Eigen::Matrix<Scalar_t, 3, 1>(msg.x, msg.y, msg.z);
    }

    template <typename Derived>
    geometry_msgs::Point point_eig2tf(const Eigen::DenseBase<Derived>& v) {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

        geometry_msgs::Point msg;
        msg.x = v.x();
        msg.y = v.y();
        msg.z = v.z();
        return msg;
    }

    template <typename Scalar_t = double>
    Eigen::Quaternion<Scalar_t> quad_tf2eig(const geometry_msgs::Quaternion& msg) {
        return Eigen::Quaternion<Scalar_t>(msg.w, msg.x, msg.y, msg.z);
    }

    template <typename Scalar_t>
    geometry_msgs::Quaternion quad_eig2tf(const Eigen::Quaternion<Scalar_t>& q) {
        geometry_msgs::Quaternion msg;
        msg.x = q.x();
        msg.y = q.y();
        msg.z = q.z();
        msg.w = q.w();
        return msg;
    }

}
#endif //FLIGHTCONTROLLER_MSG_UTILS_H
