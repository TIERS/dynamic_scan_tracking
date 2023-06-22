// function taken from https://github.com/Livox-SDK/livox_mapping/blob/master/src/livox_repub.cpp

#ifndef UTILS
#define UTILS

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include "Eigen/Dense"
#include <Eigen/Core>
#include <Eigen/LU>

#include <boost/math/tools/minima.hpp>

#include <iostream>

uint64_t TO_MERGE_CNT = 1;

using Eigen::MatrixXd;
using Eigen::VectorXd;

inline void livoxMsgToPCL(const livox_ros_driver::CustomMsgConstPtr& livox_msg_ptr,
                          pcl::PointCloud<pcl::PointXYZINormal>::Ptr& pcl_ptr,
                          std::vector<livox_ros_driver::CustomMsgConstPtr>& livox_data)
{

    livox_data.push_back(livox_msg_ptr);
    if (livox_data.size() < TO_MERGE_CNT) return;

    for (size_t j = 0; j < livox_data.size(); j++) {
        auto& livox_msg = livox_data[j];
        auto time_end = livox_msg->points.back().offset_time;
        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            pcl::PointXYZINormal pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            // float s = livox_msg->points[i].offset_time / (float)time_end;

            pt.intensity = livox_msg->points[i].line + livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
            // pt.curvature = s*0.1;
            pt.curvature = livox_msg->timebase; // save timestamp last point into curvature to compute weights for pose calculation
            pcl_ptr->push_back(pt);
        }
    }
}

void inline fuseValuesICI(VectorXd& pose_a, MatrixXd& cov_a, VectorXd& pose_b, MatrixXd& cov_b,
                          VectorXd& pose_fused, MatrixXd& cov_fused, double& omega)
{
    // Define a lambda function to compute the trace of the inverse of the sum of two matrices
    auto f = [&](double w){return ((cov_a.inverse() + cov_b.inverse() - (w * cov_a + (1 - w) * cov_b).inverse()).inverse()).trace();};

    // Compute the optimal mixing ratio (omega) that minimizes the function f
    omega = boost::math::tools::brent_find_minima(f, 0.0, 1.0, 20).first;

    // Compute the fused covariance using the optimal mixing ratio (omega)
    cov_fused = (cov_a.inverse() + cov_b.inverse() - (omega * cov_a + (1 - omega) * cov_b).inverse()).inverse();

    // Compute the gains
    MatrixXd KICI = cov_fused * (cov_a.inverse() - omega * (omega * cov_a + (1 - omega) * cov_b).inverse());
    MatrixXd LICI = cov_fused * (cov_b.inverse() - (1 - omega) * (omega * cov_a + (1 - omega) * cov_b).inverse());

    // Compute the fused pose by weighting the input poses with the computed gains
    pose_fused = KICI * pose_a + LICI * pose_b;

    // Update the input poses and covariances with the fused pose and covariance
    pose_a = pose_fused;
    pose_b = pose_fused;
    cov_a = cov_fused;
    cov_b = cov_fused;
}

#endif // UTILS

