/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Created Date: 2020-02-28 19:13:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:11:43
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class LoopPose {
public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace lidar_localization

#endif