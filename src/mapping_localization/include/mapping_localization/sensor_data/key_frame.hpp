/*
 * @Description: 关键帧，在各个模块之间传递数据
 * @Created Date: 2020-02-28 19:13:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace mapping_localization {
class KeyFrame {
public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

public:
    Eigen::Quaternionf GetQuaternion();
};
}  // namespace mapping_localization

#endif