/*
 * @Description: 打印信息
 * @Created Date: 2020-03-02 23:25:26
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TOOLS_PRINT_INFO_HPP_
#define MAPPING_LOCALIZATION_TOOLS_PRINT_INFO_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <string>

#include "pcl/common/eigen.h"

namespace mapping_localization {
class PrintInfo {
public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}  // namespace mapping_localization
#endif