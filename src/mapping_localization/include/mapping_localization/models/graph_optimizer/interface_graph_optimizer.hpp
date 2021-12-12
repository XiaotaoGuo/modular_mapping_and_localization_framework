/*
 * @Description:
 * @Created Date: 2020-03-01 18:35:19
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-12 17:22:11
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <deque>
#include <string>

namespace mapping_localization {
// 优化选项
class GraphOptimizerConfig {
public:
    GraphOptimizerConfig() {
        odom_edge_noise.resize(6);
        close_loop_noise.resize(6);
        gnss_noise.resize(3);
    }

public:
    bool use_gnss = true;
    bool use_loop_close = false;

    Eigen::VectorXd odom_edge_noise;
    Eigen::VectorXd close_loop_noise;
    Eigen::VectorXd gnss_noise;

    int optimize_step_with_key_frame = 100;
    int optimize_step_with_gnss = 100;
    int optimize_step_with_loop = 10;
};

class InterfaceGraphOptimizer {
public:
    virtual ~InterfaceGraphOptimizer() {}
    // 优化
    virtual bool Optimize() = 0;
    // 输入、输出数据
    virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose) = 0;
    virtual int GetNodeNum() = 0;
    // 添加节点、边、鲁棒核
    virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;

    ///
    ///@brief 加入 SE3 位姿节点
    ///
    ///@param pose
    ///@param need_fix
    ///
    virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;

    ///
    ///@brief 在两个节点之间加入相对位姿约束
    ///
    ///@param vertex_index1
    ///@param vertex_index2
    ///@param relative_pose
    ///@param noise
    ///
    virtual void AddSe3Edge(int vertex_index1,
                            int vertex_index2,
                            const Eigen::Isometry3d &relative_pose,
                            const Eigen::VectorXd noise) = 0;
    ///
    ///@brief 加入先验位姿边，用于加入 GPS 位置约束
    ///
    ///@param se3_vertex_index
    ///@param xyz
    ///@param noise
    ///
    virtual void AddSe3PriorXYZEdge(int se3_vertex_index, const Eigen::Vector3d &xyz, Eigen::VectorXd noise) = 0;

    // 设置优化参数
    void SetMaxIterationsNum(int max_iterations_num);

protected:
    int max_iterations_num_ = 512;
};
}  // namespace mapping_localization
#endif