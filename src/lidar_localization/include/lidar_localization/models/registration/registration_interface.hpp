/*
 * @Description: 点云匹配模块的基类
 * @Created Date: 2020-02-08 21:25:11
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:53:28
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
public:
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData::Cloud_Ptr& input_target) = 0;
    virtual bool ScanMatch(const CloudData::Cloud_Ptr& input_source,
                           const Eigen::Matrix4f& predict_pose,
                           CloudData::Cloud_Ptr& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) = 0;
    virtual float GetFitnessScore() = 0;
};
}  // namespace lidar_localization

#endif