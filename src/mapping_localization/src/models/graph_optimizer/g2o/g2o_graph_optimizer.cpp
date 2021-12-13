/*
 * @Description: 基于 g2o 的图优化器实现
 * @Created Date: 2020-03-01 18:07:42
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-13 16:36:43
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"

#include "glog/logging.h"
#include "mapping_localization/tools/tic_toc.hpp"

#include <g2o/types/slam3d/edge_se3_xyzprior.h>

namespace mapping_localization {
G2oGraphOptimizer::G2oGraphOptimizer(const YAML::Node &config_node) {
    try_load_param(config_node, "verbose", verbose_, false);

    graph_ptr_.reset(new g2o::SparseOptimizer());

    // TODO 这里使用一个工厂函数同时初始化迭代方式和线性求解方式，后续可以将迭代策略和求解方式分开
    g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;

    if (verbose_) {
        LOG(INFO) << "G2O 可用求解器类型：\n";
        solver_factory->listSolvers(LOG(INFO));
    }

    // 根据给定的求解器类型初始化求解器，包含迭代方式和线性方程组求解方式
    std::string solver_type;
    g2o::OptimizationAlgorithm *solver = nullptr;

    if (!try_load_param(config_node, "solver_type", solver_type, std::string("lm_var_cholmod"))) {
        LOG(INFO) << "Specified solver type not found, using default: lm_var_cholmod";
        solver = solver_factory->construct(solver_type, solver_property);
    } else {
        LOG(INFO) << "using solver type: " << solver_type;
        solver = solver_factory->construct(solver_type, solver_property);
    }

    graph_ptr_->setAlgorithm(solver);

    if (!graph_ptr_->solver()) {
        LOG(ERROR) << "G2O 优化器创建失败！";
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

bool G2oGraphOptimizer::Optimize() {
    static int optimize_cnt = 0;
    if (graph_ptr_->edges().size() < 1) {
        return false;
    }

    TicToc optimize_time;
    graph_ptr_->initializeOptimization();
    graph_ptr_->computeInitialGuess();
    graph_ptr_->computeActiveErrors();
    graph_ptr_->setVerbose(verbose_);

    double chi2 = graph_ptr_->chi2();
    int iterations = graph_ptr_->optimize(max_iterations_num_);

    LOG(INFO) << std::endl
              << "------ 完成第 " << ++optimize_cnt << " 次后端优化 -------" << std::endl
              << "顶点数：" << graph_ptr_->vertices().size() << ", 边数： " << graph_ptr_->edges().size() << std::endl
              << "迭代次数： " << iterations << "/" << max_iterations_num_ << std::endl
              << "用时：" << optimize_time.toc() << std::endl
              << "优化前后误差变化：" << chi2 << "--->" << graph_ptr_->chi2() << std::endl
              << std::endl;

    return true;
}

bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) {
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();

    for (int i = 0; i < vertex_num; i++) {
        g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(i));
        Eigen::Isometry3d pose = v->estimate();
        optimized_pose.push_back(pose.matrix().cast<float>());
    }

    return true;
}

int G2oGraphOptimizer::GetNodeNum() { return graph_ptr_->vertices().size(); }

void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) {
    g2o::VertexSE3 *vertex(new g2o::VertexSE3());
    vertex->setId(graph_ptr_->vertices().size());
    vertex->setEstimate(pose);
    if (need_fix) {
        vertex->setFixed(true);
    }
    graph_ptr_->addVertex(vertex);
}

void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) {
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
    need_robust_kernel_ = true;
}

void G2oGraphOptimizer::AddSe3Edge(int vertex_index1,
                                   int vertex_index2,
                                   const Eigen::Isometry3d &relative_pose,
                                   const Eigen::Vector3d &translation_noise,
                                   const Eigen::Vector3d &rotation_noise) {
    Eigen::Matrix<double, 6, 1> noise;
    noise << translation_noise, rotation_noise;
    Eigen::Matrix<double, 6, 6> information_matrix = CalculateSe3EdgeInformationMatrix(noise);
    g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3 *v2 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index2));

    g2o::EdgeSE3 *edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);
    if (need_robust_kernel_) {
        AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }
}

void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge,
                                        const std::string &kernel_type,
                                        double kernel_size) {
    if (kernel_type == "NONE") {
        return;
    }

    g2o::RobustKernel *kernel = robust_kernel_factory_->construct(kernel_type);
    if (kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);
}

void G2oGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                           const Eigen::Vector3d &xyz,
                                           const Eigen::Vector3d &translation_noise) {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(translation_noise);
    g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);
}

}  // namespace mapping_localization
