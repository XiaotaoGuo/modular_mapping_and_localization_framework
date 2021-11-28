/*
 * @Description:
 * @Created Date: 2020-02-28 19:17:00
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:40:01
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}  // namespace lidar_localization