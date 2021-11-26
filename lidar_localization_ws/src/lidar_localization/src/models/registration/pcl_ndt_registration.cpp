/*
 * @Description: NDT 匹配模块
 * @Created Date: 2020-02-08 21:46:45
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 20:50:18
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/models/registration/pcl_ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

PCLNDTRegistration::PCLNDTRegistration(const YAML::Node& node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::Point, CloudData::Point>()) {
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

PCLNDTRegistration::PCLNDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::Point, CloudData::Point>()) {
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool PCLNDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}

bool PCLNDTRegistration::SetInputTarget(const CloudData::Cloud_Ptr& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool PCLNDTRegistration::ScanMatch(const CloudData::Cloud_Ptr& input_source,
                                   const Eigen::Matrix4f& predict_pose,
                                   CloudData::Cloud_Ptr& result_cloud_ptr,
                                   Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}

float PCLNDTRegistration::GetFitnessScore() { return ndt_ptr_->getFitnessScore(); }
}  // namespace lidar_localization