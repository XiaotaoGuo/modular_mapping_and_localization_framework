/*
 * @Description: 基于 ceres 的优化器定义
 * @Created Date: 2021-12-09 23:24:10
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-15 12:11:17
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/graph_optimizer/ceres/ceres_graph_optimizer.hpp"

namespace mapping_localization {

CeresGraphOptimizer::CeresGraphOptimizer(const YAML::Node &config_node) {
    try_load_param(config_node, "verbose", verbose_, false);

    std::string linear_solver_type;
    std::string optimization_strategy;

    try_load_param(config_node, "linear_solver_type", linear_solver_type, linear_solver_set_[0], linear_solver_set_);
    try_load_param(config_node,
                   "optimization_strategy",
                   optimization_strategy,
                   optimization_strategy_set_[0],
                   optimization_strategy_set_);

    ceres::StringToLinearSolverType(linear_solver_type, &options_.linear_solver_type);
    ceres::StringToTrustRegionStrategyType(optimization_strategy, &options_.trust_region_strategy_type);
    quaternion_local_parameterization_ = new ceres::EigenQuaternionParameterization;
}

CeresGraphOptimizer::~CeresGraphOptimizer() {
    if (quaternion_local_parameterization_) {
        delete quaternion_local_parameterization_;
    }

    if (loss_function_) {
        delete loss_function_;
    }
}

bool CeresGraphOptimizer::Optimize() {
    ceres::Solve(options_, &problem_, &summary_);
    if (verbose_) {
        LOG(INFO) << summary_.FullReport();
    }

    // LOG(INFO) << std::endl
    //           << "------ 完成第 " << ++optimize_cnt << " 次后端优化 -------" << std::endl
    //           << "顶点数：" << graph_ptr_->vertices().size() << ", 边数： " << graph_ptr_->edges().size() <<
    //           std::endl
    //           << "迭代次数： " << iterations << "/" << max_iterations_num_ << std::endl
    //           << "用时：" << optimize_time.toc() << std::endl
    //           << "优化前后误差变化：" << chi2 << "--->" << graph_ptr_->chi2() << std::endl
    //           << std::endl;

    return true;
}

bool CeresGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) {
    optimized_pose.resize(vertices_.size());

    for (size_t i = 0; i < vertices_.size(); ++i) {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = vertices_[i].quat.toRotationMatrix();
        pose.block<3, 1>(0, 3) = vertices_[i].pos;
        optimized_pose[i] = pose.template cast<float>();
    }

    return true;
}

int CeresGraphOptimizer::GetNodeNum() { return vertices_.size(); }

void CeresGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) {
    need_robust_kernel_ = true;
    loss_function_ = new ceres::HuberLoss(1.0);
}

void CeresGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) {
    vertices_.push_back(PoseSE3());
    PoseSE3 &pose_se3 = vertices_.back();
    pose_se3.pos = pose.translation();
    pose_se3.quat = pose.rotation();

    problem_.AddParameterBlock(pose_se3.pos.data(), 3);
    problem_.AddParameterBlock(pose_se3.quat.coeffs().data(), 4);

    problem_.SetParameterization(pose_se3.quat.coeffs().data(), quaternion_local_parameterization_);

    if (need_fix) {
        problem_.SetParameterBlockConstant(vertices_.back().pos.data());
        problem_.SetParameterBlockConstant(vertices_.back().quat.coeffs().data());
    }
}

void CeresGraphOptimizer::AddSe3Edge(int vertex_index1,
                                     int vertex_index2,
                                     const Eigen::Isometry3d &relative_pose,
                                     const Eigen::Vector3d &translation_noise,
                                     const Eigen::Vector3d &rotation_noise) {
    PoseSE3 &v1 = vertices_[vertex_index1];
    PoseSE3 &v2 = vertices_[vertex_index2];

    PoseSE3 measurement;
    measurement.pos = relative_pose.translation();
    measurement.quat = relative_pose.rotation();

    Eigen::Matrix<double, 6, 1> noise;
    noise << translation_noise, rotation_noise;
    Eigen::Matrix<double, 6, 6> sqrt_info = CalculateSqrtDiagMatrix(noise);

    ceres::CostFunction *cost_function = RelativePoseEdge::Create(measurement, sqrt_info);

    problem_.AddResidualBlock(
        cost_function, loss_function_, v1.pos.data(), v1.quat.coeffs().data(), v2.pos.data(), v2.quat.coeffs().data());
}

void CeresGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                             const Eigen::Vector3d &xyz,
                                             const Eigen::Vector3d &translation_noise) {
    PoseSE3 &v = vertices_[se3_vertex_index];

    Eigen::Matrix3d prior_sqrt_info = CalculateSqrtDiagMatrix(translation_noise);

    ceres::CostFunction *cost_function = PriorPositionEdge::create(xyz, prior_sqrt_info);

    problem_.AddResidualBlock(cost_function, nullptr, v.pos.data());
}

}  // namespace mapping_localization
