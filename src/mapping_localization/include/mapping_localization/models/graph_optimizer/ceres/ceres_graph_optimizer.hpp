/*
 * @Description: 基于 ceres 的优化器声明
 * @Created Date: 2021-12-07 12:41:27
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-13 17:38:26
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_CERES_GRAPH_OPTIMIZER_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_CERES_GRAPH_OPTIMIZER_HPP_

#include <ceres/ceres.h>

#include "mapping_localization/models/graph_optimizer/ceres/edges/prior_position_edge.hpp"
#include "mapping_localization/models/graph_optimizer/ceres/edges/relative_pose_edge.hpp"
#include "mapping_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

#include "mapping_localization/tools/utils.hpp"

namespace mapping_localization {

class CeresGraphOptimizer : public InterfaceGraphOptimizer {
public:
    CeresGraphOptimizer(const YAML::Node &config_node);

    ~CeresGraphOptimizer();

    bool Optimize() override;

    bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) override;
    int GetNodeNum() override;

    // 添加节点、边、鲁棒核
    void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
    void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) override;
    void AddSe3Edge(int vertex_index1,
                    int vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::Vector3d &translation_noise,
                    const Eigen::Vector3d &rotation_noise) override;
    void AddSe3PriorXYZEdge(int se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            const Eigen::Vector3d &translation_noise) override;

private:
    ceres::Problem problem_;
    ceres::Solver::Options options_;
    ceres::Solver::Summary summary_;

    ceres::LocalParameterization *quaternion_local_parameterization_ = nullptr;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;
    ceres::LossFunction *loss_function_ = nullptr;

    bool verbose_ = false;

    std::deque<PoseSE3> vertices_;

    const std::vector<std::string> linear_solver_set_ = {
        ceres::LinearSolverTypeToString(ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY),
        ceres::LinearSolverTypeToString(ceres::LinearSolverType::DENSE_QR),
        ceres::LinearSolverTypeToString(ceres::LinearSolverType::DENSE_SCHUR),
        ceres::LinearSolverTypeToString(ceres::LinearSolverType::SPARSE_SCHUR)};
    const std::vector<std::string> optimization_strategy_set_ = {
        ceres::TrustRegionStrategyTypeToString(ceres::TrustRegionStrategyType::DOGLEG),
        ceres::TrustRegionStrategyTypeToString(ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT)};
};

}  // namespace mapping_localization

#endif
