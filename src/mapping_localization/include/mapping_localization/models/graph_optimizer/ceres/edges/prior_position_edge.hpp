/*
 * @Description:
 * @Created Date: 2021-12-11 19:22:35
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-12 20:11:17
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_EDGES_PRIOR_POSITION_EDGE_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_EDGES_PRIOR_POSITION_EDGE_HPP_

#include "mapping_localization/models/graph_optimizer/ceres/nodes/pose_se3.hpp"

namespace mapping_localization {
class PriorPositionEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PriorPositionEdge(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& sqrt_info)
        : measurement_(measurement), sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const v1, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> estimate(v1);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_eigen(residual);
        residual_eigen = estimate - measurement_;
        residual_eigen.applyOnTheLeft(sqrt_info_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& sqrt_info) {
        return (
            new ceres::AutoDiffCostFunction<PriorPositionEdge, 3, 3>(new PriorPositionEdge(measurement, sqrt_info)));
    }

private:
    Eigen::Vector3d measurement_;
    Eigen::Matrix3d sqrt_info_;
};
}  // namespace mapping_localization

#endif
