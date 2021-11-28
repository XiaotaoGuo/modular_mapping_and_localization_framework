/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef MAPPING_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define MAPPING_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>

#include "mapping_localization/models/registration/registration_interface.hpp"

namespace mapping_localization {
class NDTRegistration : public RegistrationInterface {
public:
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

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
}  // namespace mapping_localization

#endif