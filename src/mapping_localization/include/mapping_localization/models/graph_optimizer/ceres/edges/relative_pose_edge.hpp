/*
 * @Description:
 * @Created Date: 2021-12-11 17:28:38
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-12 19:17:01
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_EDGES_RELATIVE_POSE_EDGE_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_EDGES_RELATIVE_POSE_EDGE_HPP_

#include "mapping_localization/models/graph_optimizer/ceres/nodes/pose_se3.hpp"

namespace mapping_localization {
class RelativePoseEdge {
public:
    RelativePoseEdge(const PoseSE3& measurement, const Eigen::Matrix<double, 6, 6>& sqrt_information)
        : measurement_(measurement), sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T* const p_a_ptr,
                    const T* const q_a_ptr,
                    const T* const p_b_ptr,
                    const T* const q_b_ptr,
                    T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion<T> delta_q = measurement_.quat.template cast<T>() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_ab_estimated - measurement_.pos.template cast<T>();
        residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const PoseSE3& measurement,
                                       const Eigen::Matrix<double, 6, 6>& sqrt_information) {
        return new ceres::AutoDiffCostFunction<RelativePoseEdge, 6, 3, 4, 3, 4>(
            new RelativePoseEdge(measurement, sqrt_information));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The measurement for the position of B relative to A in the A frame.
    const PoseSE3 measurement_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix<double, 6, 6> sqrt_information_;
};

}  // namespace mapping_localization

#endif
