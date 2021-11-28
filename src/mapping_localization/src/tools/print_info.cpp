/*
 * @Description:
 * @Created Date: 2020-03-02 23:28:54
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:44:03
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/tools/print_info.hpp"

#include "glog/logging.h"

namespace mapping_localization {
void PrintInfo::PrintPose(std::string head, Eigen::Matrix4f pose) {
    Eigen::Affine3f aff_pose;
    aff_pose.matrix() = pose;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
    std::cout << head << x << "," << y << "," << z << "," << roll * 180 / M_PI << ","
              << pitch * 180 / M_PI << "," << yaw * 180 / M_PI << std::endl;
}
}  // namespace mapping_localization