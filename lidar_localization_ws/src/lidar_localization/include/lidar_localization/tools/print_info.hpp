/*
 * @Description: 打印信息
 * @Created Date: 2020-03-02 23:25:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:31:33
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_TOOLS_PRINT_INFO_HPP_
#define LIDAR_LOCALIZATION_TOOLS_PRINT_INFO_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <string>

#include "pcl/common/eigen.h"

namespace lidar_localization {
class PrintInfo {
public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}  // namespace lidar_localization
#endif