/*
 * @Description: 点云畸变补偿
 * @Created Date: 2020-02-25 14:38:12
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define MAPPING_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"
#include "mapping_localization/models/scan_adjust/distortion_adjust.hpp"
#include "mapping_localization/sensor_data/cloud_data.hpp"
#include "mapping_localization/sensor_data/velocity_data.hpp"

namespace mapping_localization {
class DistortionAdjust {
public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(CloudData::Cloud_Ptr& input_cloud_ptr, CloudData::Cloud_Ptr& output_cloud_ptr);

private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
}  // namespace mapping_localization
#endif