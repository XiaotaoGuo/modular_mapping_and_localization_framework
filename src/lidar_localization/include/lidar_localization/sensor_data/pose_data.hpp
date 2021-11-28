/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Created Date: 2020-02-27 23:10:56
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:11:45
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class PoseData {
public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;

public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_localization

#endif