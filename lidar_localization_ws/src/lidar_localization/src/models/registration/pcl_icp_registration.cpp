/*
 * @Description:
 * @Created Date: 2021-11-24 23:36:42
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-25 00:42:19
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/models/registration/pcl_icp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

PCLICPRegistration::PCLICPRegistration(const YAML::Node& node)
    : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    float max_correspodence_dist = node["max_correspodence_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float fitness_eps = node["fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

PCLICPRegistration::PCLICPRegistration(float max_correspodence_dist,
                                       float trans_eps,
                                       float fitness_eps,
                                       int max_iter)
    : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

bool PCLICPRegistration::SetRegistraionParam(float max_correspodence_dist,
                                             float trans_eps,
                                             float fitness_eps,
                                             int max_iter) {
    icp_ptr_->setMaxCorrespondenceDistance(max_correspodence_dist);
    icp_ptr_->setMaximumIterations(max_iter);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setEuclideanFitnessEpsilon(fitness_eps);

    std::cout << "ICP 的匹配参数为：" << std::endl
              << "max_correspodence_dist: " << max_correspodence_dist << ", "
              << "fitness_eps: " << fitness_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}

bool PCLICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);
    return true;
}

bool PCLICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                   const Eigen::Matrix4f& predict_pose,
                                   CloudData::CLOUD_PTR& result_cloud_ptr,
                                   Eigen::Matrix4f& result_pose) {
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}

float PCLICPRegistration::GetFitnessScore() { return icp_ptr_->getFitnessScore(); }
}  // namespace lidar_localization
