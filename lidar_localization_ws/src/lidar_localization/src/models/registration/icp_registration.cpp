/*
 * @Description:
 * @Created Date: 2021-11-25 19:10:58
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-26 00:30:16
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/models/registration/icp_registration.hpp"

#include <pcl/common/transforms.h>
#include <Eigen/SVD>

#include "glog/logging.h"

namespace lidar_localization {

ICPRegistration::ICPRegistration(const YAML::Node& node)
    : target_kd_tree_ptr_(new pcl::KdTreeFLANN<CloudData::Point>()) {
    float max_correspodence_dist = node["max_correspodence_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float fitness_eps = node["fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

ICPRegistration::ICPRegistration(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter)
    : target_kd_tree_ptr_(new pcl::KdTreeFLANN<CloudData::Point>()) {
    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

bool ICPRegistration::SetRegistraionParam(float max_correspodence_dist,
                                          float trans_eps,
                                          float fitness_eps,
                                          int max_iter) {
    param_.max_correspodence_dist = max_correspodence_dist;
    param_.trans_eps = trans_eps;
    param_.fitness_eps = fitness_eps;
    param_.max_iter = max_iter;

    std::cout << "ICP 的匹配参数为：" << std::endl
              << "max_correspodence_dist: " << max_correspodence_dist << ", "
              << "fitness_eps: " << fitness_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::Cloud_Ptr& input_target) {
    target_kd_tree_ptr_->setInputCloud(input_target);
    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::Cloud_Ptr& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::Cloud_Ptr& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    Eigen::Matrix4f esitimated_pose = predict_pose;
    bool converge = false;
    int iter = 0;

    // keep updating pose until converge or reach max iteration
    while (iter < param_.max_iter && !converge) {
        iter++;

        CloudData::Cloud transformed_cloud;
        pcl::transformPointCloud(*input_source, transformed_cloud, esitimated_pose);

        std::vector<float> source_points_xyz;
        std::vector<float> target_points_xyz;

        source_points_xyz.reserve(transformed_cloud.points.size() * 3);  //
        target_points_xyz.reserve(transformed_cloud.points.size() * 3);

        float score = 0.0;
        for (size_t i = 0; i < input_source->points.size(); ++i) {
            const CloudData::Point& transformed_point = transformed_cloud.points[i];

            std::vector<int> indices(1);
            std::vector<float> distances(1);

            target_kd_tree_ptr_->nearestKSearch(transformed_point, 1, indices, distances);

            score += distances[0];

            if (distances[0] > param_.max_correspodence_dist) continue;

            source_points_xyz.push_back(input_source->points[i].x);
            source_points_xyz.push_back(input_source->points[i].y);
            source_points_xyz.push_back(input_source->points[i].z);

            target_points_xyz.push_back(target_kd_tree_ptr_->getInputCloud()->points[indices[0]].x);
            target_points_xyz.push_back(target_kd_tree_ptr_->getInputCloud()->points[indices[0]].y);
            target_points_xyz.push_back(target_kd_tree_ptr_->getInputCloud()->points[indices[0]].z);
        }

        // condition 1: fitness score converge
        score /= (transformed_cloud.points.size());
        if (std::fabs(score_ - score) < param_.fitness_eps) {
            converge = true;
            break;
        }
        score_ = score;

        Eigen::Map<Eigen::Matrix3Xf> source_points(source_points_xyz.data(), 3, source_points_xyz.size() / 3);
        Eigen::Map<Eigen::Matrix3Xf> target_points(target_points_xyz.data(), 3, target_points_xyz.size() / 3);

        Eigen::Vector3f source_mean = source_points.rowwise().mean();
        Eigen::Vector3f target_mean = target_points.rowwise().mean();

        source_points.colwise() -= source_mean;
        target_points.colwise() -= target_mean;

        Eigen::Matrix3f H = source_points * target_points.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector3f prev_translation = esitimated_pose.block<3, 1>(0, 3);

        Eigen::Matrix3f updated_rotation = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector3f updated_translation = target_mean - updated_rotation * source_mean;

        esitimated_pose.block<3, 3>(0, 0) = updated_rotation;
        esitimated_pose.block<3, 1>(0, 3) = updated_translation;

        // condition 2: translation converge
        if ((updated_translation - prev_translation).squaredNorm() < param_.trans_eps) {
            converge = true;
        }
    }

    // used final mean distance (across all points) as fitness score
    result_pose = esitimated_pose;
    score_ = 0.0;
    CloudData::Cloud final_transformed_cloud;
    pcl::transformPointCloud(*input_source, final_transformed_cloud, esitimated_pose);
    for (const auto& pt : final_transformed_cloud.points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        target_kd_tree_ptr_->nearestKSearch(pt, 1, indices, distances);

        score_ += distances[0];
    }
    score_ /= final_transformed_cloud.points.size();
    return true;
}

float ICPRegistration::GetFitnessScore() { return score_; }
}  // namespace lidar_localization
