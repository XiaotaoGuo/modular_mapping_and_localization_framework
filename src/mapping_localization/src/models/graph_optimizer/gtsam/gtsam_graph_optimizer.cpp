/*
 * @Description:
 * @Created Date: 2021-12-12 21:14:14
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-13 17:37:24
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/graph_optimizer/gtsam/gtsam_graph_optimizer.hpp"

namespace mapping_localization {

GTSamGraphOptimizer::GTSamGraphOptimizer(const YAML::Node &config_node) {
    try_load_param(config_node, "verbose", verbose_, verbose_set_[0], verbose_set_);
    try_load_param(config_node,
                   "optimization_strategy",
                   optimization_strategy_,
                   optimization_strategy_set_[0],
                   optimization_strategy_set_);

    try_load_param(config_node, "linear_solver_type", linear_solver_type_, linear_solver_set_[0], linear_solver_set_);
}

bool GTSamGraphOptimizer::Optimize() {
    if (optimization_strategy_ == "LevenbergMarquardt") {
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity(verbose_);
        params.setLinearSolverType(linear_solver_type_);
        estimates_ = gtsam::LevenbergMarquardtOptimizer(graph_, estimates_, params).optimize();
    } else if (optimization_strategy_ == "GaussianNewton") {
        gtsam::GaussNewtonParams params;
        params.setVerbosity(verbose_);
        params.setLinearSolverType(linear_solver_type_);
        estimates_ = gtsam::GaussNewtonOptimizer(graph_, estimates_, params).optimize();
    } else if (optimization_strategy_ == "GaussianNewton") {
        gtsam::DoglegParams params;
        params.setVerbosity(verbose_);
        params.setLinearSolverType(linear_solver_type_);
        estimates_ = gtsam::DoglegOptimizer(graph_, estimates_, params).optimize();
    }

    return true;
}

bool GTSamGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) {
    optimized_pose.resize(estimates_.size());
    for (size_t i = 0; i < estimates_.size(); ++i) {
        const gtsam::Pose3 &p = estimates_.at<gtsam::Pose3>(i);
        optimized_pose[i] = p.matrix().matrix().template cast<float>();
    }

    return true;
}

int GTSamGraphOptimizer::GetNodeNum() { return estimates_.size(); }

// 添加节点、边、鲁棒核
void GTSamGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) {
    need_robust_kernel_ = true;
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
}

void GTSamGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) {
    gtsam::Key key = estimates_.size();
    gtsam::Pose3 p(pose.matrix());
    estimates_.insert(key, p);

    // for fixed vertex, add a prior with pretty small noise
    if (need_fix) {
        gtsam::Vector6 fixed_noise;
        fixed_noise << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(fixed_noise);
        graph_.addPrior(key, p, prior_noise);
    }
}
void GTSamGraphOptimizer::AddSe3Edge(int vertex_index1,
                                     int vertex_index2,
                                     const Eigen::Isometry3d &relative_pose,
                                     const Eigen::Vector3d &translation_noise,
                                     const Eigen::Vector3d &rotation_noise) {
    gtsam::Pose3 measurement(relative_pose.matrix());

    Eigen::Matrix<double, 6, 1> noise;
    noise << rotation_noise, translation_noise;
    gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6(noise));

    if (need_robust_kernel_) {
        gtsam::noiseModel::Robust::shared_ptr robust_loss;
        if (robust_kernel_name_ == "Huber") {
            robust_loss = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(robust_kernel_size_), noise_model);
        }
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            vertex_index1, vertex_index2, measurement, robust_loss);
    } else {
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            vertex_index1, vertex_index2, measurement, noise_model);
    }
}

void GTSamGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                             const Eigen::Vector3d &xyz,
                                             const Eigen::Vector3d &translation_noise) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(translation_noise);
    gtsam::Matrix3 information(information_matrix.matrix());
    gtsam::Point3 measurement(xyz.x(), xyz.y(), xyz.z());
    graph_.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose3>>(
        se3_vertex_index, measurement, gtsam::noiseModel::Gaussian::Information(information));
}

}  // namespace mapping_localization
