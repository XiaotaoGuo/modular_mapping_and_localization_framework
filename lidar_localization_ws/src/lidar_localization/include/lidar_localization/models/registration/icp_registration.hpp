/*
 * @Description:
 * @Created Date: 2021-11-25 19:07:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-25 20:47:10
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

// #include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {

class ICPRegistration : public RegistrationInterface {
public:
    struct Param {
        float max_correspodence_dist = 0.0;
        float trans_eps = 0.0;
        float fitness_eps = 0.0;
        int max_iter = 0;
    };

public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter);

    bool SetInputTarget(const CloudData::Cloud_Ptr& input_target) override;
    bool ScanMatch(const CloudData::Cloud_Ptr& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::Cloud_Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistraionParam(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter);

private:
    pcl::KdTreeFLANN<CloudData::Point>::Ptr target_kd_tree_ptr_;
    Param param_;
    float score_ = std::numeric_limits<float>::max();
};

}  // namespace lidar_localization

#endif