/*
 * @Description:
 * @Created Date: 2021-11-24 23:24:45
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-25 20:47:24
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
    PCLICPRegistration(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter);

    bool SetInputTarget(const CloudData::Cloud_Ptr& input_target) override;
    bool ScanMatch(const CloudData::Cloud_Ptr& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::Cloud_Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistraionParam(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter);

private:
    pcl::IterativeClosestPoint<CloudData::Point, CloudData::Point>::Ptr icp_ptr_;
};

}  // namespace lidar_localization

#endif