/*
 * @Description: 不滤波
 * @Created Date: 2020-02-09 19:53:20
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:53:50
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/models/cloud_filter/no_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {
NoFilter::NoFilter() {}

bool NoFilter::Filter(const CloudData::Cloud_Ptr& input_cloud_ptr,
                      CloudData::Cloud_Ptr& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::Cloud(*input_cloud_ptr));
    return true;
}
}  // namespace lidar_localization