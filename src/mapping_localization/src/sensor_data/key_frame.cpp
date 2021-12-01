/*
 * @Description:
 * @Created Date: 2020-02-28 19:17:00
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-30 11:15:42
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);

    return q;
}
}  // namespace mapping_localization