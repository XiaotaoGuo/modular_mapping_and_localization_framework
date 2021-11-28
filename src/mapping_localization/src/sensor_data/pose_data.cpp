/*
 * @Description:
 * @Created Date: 2020-02-28 18:50:16
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:40:26
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/sensor_data/pose_data.hpp"

namespace mapping_localization {
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}  // namespace mapping_localization