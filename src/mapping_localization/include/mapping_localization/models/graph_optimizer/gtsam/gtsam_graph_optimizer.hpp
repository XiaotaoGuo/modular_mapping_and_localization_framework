/*
 * @Description:
 * @Created Date: 2021-12-12 20:51:44
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-13 17:33:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_GTSAM_GTSAM_GRAPH_OPTIMIZER_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_GTSAM_GTSAM_GRAPH_OPTIMIZER_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <gtsam/nonlinear/Values.h>

#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "mapping_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

#include "mapping_localization/tools/utils.hpp"

namespace mapping_localization {

class GTSamGraphOptimizer : public InterfaceGraphOptimizer {
public:
    GTSamGraphOptimizer(const YAML::Node &config_node);

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
    std::string verbose_ = "SILENT";
    std::string optimization_strategy_ = "LevenbergMarquardt";
    std::string linear_solver_type_ = "MULTIFRONTAL_CHOLESKY";

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimates_;

    bool need_robust_kernel_ = false;
    std::string robust_kernel_name_ = "Huber";
    double robust_kernel_size_ = 1.345;

    const std::vector<std::string> verbose_set_ = {"SILENT", "TERMINATION", "DELTA"};
    const std::vector<std::string> optimization_strategy_set_ = {"GaussianNewton", "LevenbergMarquardt", "Dogleg"};
    const std::vector<std::string> linear_solver_set_ = {"MULTIFRONTAL_CHOLESKY",
                                                         "MULTIFRONTAL_QR",
                                                         "SEQUENTIAL_CHOLESKY",
                                                         "SEQUENTIAL_QR"};
};
}  // namespace mapping_localization

#endif