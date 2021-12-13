/*
 * @Description:
 * @Created Date: 2020-03-01 18:07:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-13 16:26:24
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_G2O_GRAPH_OPTIMIZER_HPP_

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "mapping_localization/models/graph_optimizer/g2o/edge/edge_se3_priorquat.hpp"
#include "mapping_localization/models/graph_optimizer/g2o/edge/edge_se3_priorxyz.hpp"
#include "mapping_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

#include "mapping_localization/tools/utils.hpp"

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);

G2O_USE_OPTIMIZATION_LIBRARY(pcg)      // Preconditioned conjugate gradient Libarary
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // sparse Cholesky factorization library
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // Sparse matrix package by c

// namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
// } // namespace g2o

namespace mapping_localization {
class G2oGraphOptimizer : public InterfaceGraphOptimizer {
public:
    G2oGraphOptimizer(const YAML::Node &config_node);
    // 优化
    bool Optimize() override;
    // 输出数据
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
    void AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

private:
    g2o::RobustKernelFactory *robust_kernel_factory_;
    std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;

    bool verbose_ = false;
};
}  // namespace mapping_localization
#endif