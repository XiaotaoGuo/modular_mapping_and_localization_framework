/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Created Date: 2020-02-27 23:10:56
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace mapping_localization {
class PoseData {
public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace mapping_localization

#endif