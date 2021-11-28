/*
 * @Description:
 * @Created Date: 2020-02-28 19:17:00
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:40:12
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/sensor_data/loop_pose.hpp"

namespace mapping_localization {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}  // namespace mapping_localization