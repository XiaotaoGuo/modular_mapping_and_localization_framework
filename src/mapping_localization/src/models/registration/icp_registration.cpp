/*
 * @Description:
 * @Created Date: 2021-11-25 19:10:58
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-29 01:06:23
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/registration/icp_registration.hpp"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <sophus/so3.hpp>

namespace mapping_localization {

ICPRegistration::ICPRegistration(const YAML::Node& node) {
    // initialize match method
    std::string match_method = node["matching_method"].as<std::string>();
    InitializeMatchMethod(match_method);

    float max_correspodence_dist = node["max_correspodence_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float fitness_eps = node["fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

ICPRegistration::ICPRegistration(const std::string& match_method,
                                 float max_correspodence_dist,
                                 float trans_eps,
                                 float fitness_eps,
                                 int max_iter) {
    InitializeMatchMethod(match_method);
    SetRegistraionParam(max_correspodence_dist, trans_eps, fitness_eps, max_iter);
}

void ICPRegistration::InitializeMatchMethod(const std::string& match_method) {
    if (match_method == "PCL") {
        match_method_ = MatchMethod::PCL;
        icp_ptr_.reset(new pcl::IterativeClosestPoint<CloudData::Point, CloudData::Point>());
    } else if (match_method == "SVD") {
        match_method_ = MatchMethod::SVD;
        target_kd_tree_ptr_.reset(new pcl::KdTreeFLANN<CloudData::Point>());
    } else if (match_method == "GN") {
        match_method_ = MatchMethod::GaussianNewton;
        target_kd_tree_ptr_.reset(new pcl::KdTreeFLANN<CloudData::Point>());
    } else {
        std::cout << "找不到 " << match_method << " 匹配方法，默认使用 PCL 自带 ICP 接口" << std::endl;
        match_method_ = MatchMethod::PCL;
        icp_ptr_.reset(new pcl::IterativeClosestPoint<CloudData::Point, CloudData::Point>());
    }
}

bool ICPRegistration::SetRegistraionParam(float max_correspodence_dist,
                                          float trans_eps,
                                          float fitness_eps,
                                          int max_iter) {
    param_.max_correspodence_dist = max_correspodence_dist;
    param_.trans_eps = trans_eps;
    param_.fitness_eps = fitness_eps;
    param_.max_iter = max_iter;

    if (match_method_ == MatchMethod::PCL) {
        icp_ptr_->setMaxCorrespondenceDistance(max_correspodence_dist);
        icp_ptr_->setMaximumIterations(max_iter);
        icp_ptr_->setTransformationEpsilon(trans_eps);
        icp_ptr_->setEuclideanFitnessEpsilon(fitness_eps);
    }

    std::cout << "ICP 的匹配参数为：" << std::endl
              << "max_correspodence_dist: " << max_correspodence_dist << ", "
              << "fitness_eps: " << fitness_eps << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter << std::endl
              << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::Cloud_Ptr& input_target) {
    if (match_method_ == MatchMethod::PCL) {
        icp_ptr_->setInputTarget(input_target);
    } else {
        target_kd_tree_ptr_->setInputCloud(input_target);
    }
    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::Cloud_Ptr& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::Cloud_Ptr& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    if (match_method_ == MatchMethod::PCL) {
        return MatchWithPCL(input_source, predict_pose, result_cloud_ptr, result_pose);
    }

    if (match_method_ == MatchMethod::SVD) {
        MatchWithSVD(input_source, predict_pose, result_pose);
    } else if (match_method_ == MatchMethod::GaussianNewton) {
        MatchWithGassianNewton(input_source, predict_pose, result_pose);
    } else {
        // won't happen
        std::cout << "Matching method invalid.\n";
        return false;
    }

    score_ = 0.0;
    pcl::transformPointCloud(*input_source, *result_cloud_ptr, result_pose);
    for (const auto& pt : result_cloud_ptr->points) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);

        target_kd_tree_ptr_->nearestKSearch(pt, 1, indices, distances);

        score_ += distances[0];
    }
    score_ /= result_cloud_ptr->points.size();
    return true;
}

float ICPRegistration::GetFitnessScore() { return score_; }

bool ICPRegistration::MatchWithPCL(const CloudData::Cloud_Ptr& input_source,
                                   const Eigen::Matrix4f& predict_pose,
                                   CloudData::Cloud_Ptr& result_cloud_ptr,
                                   Eigen::Matrix4f& result_pose) {
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = icp_ptr_->getFinalTransformation();
    score_ = icp_ptr_->getFitnessScore();

    return true;
}

bool ICPRegistration::MatchWithSVD(const CloudData::Cloud_Ptr& input_source,
                                   const Eigen::Matrix4f& predict_pose,
                                   Eigen::Matrix4f& result_pose)

{
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

    return true;
}

bool ICPRegistration::MatchWithGassianNewton(const CloudData::Cloud_Ptr& input_source,
                                             const Eigen::Matrix4f& predict_pose,
                                             Eigen::Matrix4f& result_pose) {
    Eigen::Matrix4f esitimated_pose = predict_pose;
    bool converge = false;
    int iter = 0;

    // keep updating pose until converge or reach max iteration
    while (iter < param_.max_iter && !converge) {
        iter++;

        CloudData::Cloud transformed_cloud;
        pcl::transformPointCloud(*input_source, transformed_cloud, esitimated_pose);

        Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();  // J^TJ
        Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();  // J^Te

        float score = 0.0;
        for (size_t i = 0; i < input_source->points.size(); ++i) {
            const CloudData::Point& transformed_point = transformed_cloud.points[i];

            std::vector<int> indices(1);
            std::vector<float> distances(1);

            target_kd_tree_ptr_->nearestKSearch(transformed_point, 1, indices, distances);

            const CloudData::Point& original_point = input_source->points[i];
            const CloudData::Point& nearest_point = target_kd_tree_ptr_->getInputCloud()->points[indices[0]];
            score += distances[0];

            if (distances[0] > param_.max_correspodence_dist) continue;

            Eigen::Vector3f source_pt(original_point.x, original_point.y, original_point.z);
            Eigen::Vector3f transformed_pt(transformed_point.x, transformed_point.y, transformed_point.z);
            Eigen::Vector3f target_pt(nearest_point.x, nearest_point.y, nearest_point.z);

            Eigen::Matrix<float, 3, 6> jacobian = Eigen::Matrix<float, 3, 6>::Zero();
            jacobian.block<3, 3>(0, 0) = -esitimated_pose.block<3, 3>(0, 0) * Sophus::SO3f::hat(source_pt);
            jacobian.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
            Eigen::Vector3f resiual = transformed_pt - target_pt;

            H += jacobian.transpose() * jacobian;
            b += -jacobian.transpose() * resiual;
        }

        // condition 1: fitness score converge
        score /= (transformed_cloud.points.size());
        if (std::fabs(score_ - score) < param_.fitness_eps) {
            converge = true;
            break;
        }
        score_ = score;

        if (H.determinant() == 0) {
            std::cout << "determinat is zero!\n";
            H += 0.1 * Eigen::Matrix<float, 6, 6>::Identity();
        }

        Eigen::Matrix<float, 6, 1> delta_x = H.householderQr().solve(b);

        esitimated_pose.block<3, 3>(0, 0) *= Sophus::SO3f::exp(delta_x.block<3, 1>(0, 0)).matrix();
        esitimated_pose.block<3, 1>(0, 3) += delta_x.block<3, 1>(3, 0);

        // condition 2: transformation converge
        if (delta_x.squaredNorm() < param_.trans_eps) {
            converge = true;
        }
    }

    // used final mean distance (across all points) as fitness score
    result_pose = esitimated_pose;

    return true;
}
}  // namespace mapping_localization