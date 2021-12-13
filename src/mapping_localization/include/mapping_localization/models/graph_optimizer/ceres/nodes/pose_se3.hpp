/*
 * @Description:
 * @Created Date: 2021-12-11 17:03:05
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-12 17:45:43
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_NODES_POSE_SE3_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_CERES_NODES_POSE_SE3_HPP_

#include <Eigen/Dense>

namespace mapping_localization {

struct PoseSE3 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
};

}  // namespace mapping_localization

#endif