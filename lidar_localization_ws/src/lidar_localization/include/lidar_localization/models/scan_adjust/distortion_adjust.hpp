/*
 * @Description: 点云畸变补偿
 * @Created Date: 2020-02-25 14:38:12
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:25:38
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class DistortionAdjust {
public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr,
                     CloudData::CLOUD_PTR& output_cloud_ptr);

private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
}  // namespace lidar_localization
#endif