/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_PCL_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_PCL_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>

#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class PCLNDTRegistration : public RegistrationInterface {
public:
    PCLNDTRegistration(const YAML::Node& node);
    PCLNDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::Cloud_Ptr& input_target) override;
    bool ScanMatch(const CloudData::Cloud_Ptr& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::Cloud_Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

private:
    pcl::NormalDistributionsTransform<CloudData::Point, CloudData::Point>::Ptr ndt_ptr_;
};
}  // namespace lidar_localization

#endif