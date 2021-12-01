/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Created Date: 2020-02-28 19:13:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-30 20:20:35
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace mapping_localization {
class LoopPose {
public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    bool confidence = false;  // false -- 回环检测到，但是地图效果不好， true -- 回环检测到，且匹配效果较好
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace mapping_localization

#endif