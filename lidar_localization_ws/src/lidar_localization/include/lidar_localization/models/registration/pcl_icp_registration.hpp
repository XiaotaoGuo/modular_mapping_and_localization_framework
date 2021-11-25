/*
 * @Description:
 * @Created Date: 2021-11-24 23:24:45
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-25 00:02:08
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_PCL_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_PCL_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>

#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {

class PCLICPRegistration : public RegistrationInterface {
public:
    PCLICPRegistration(const YAML::Node& node);
    PCLICPRegistration(float max_correspodence_dist,
                       float trans_eps,
                       float fitness_eps,
                       int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistraionParam(float max_correspodence_dist,
                             float trans_eps,
                             float fitness_eps,
                             int max_iter);

private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
};

}  // namespace lidar_localization

#endif