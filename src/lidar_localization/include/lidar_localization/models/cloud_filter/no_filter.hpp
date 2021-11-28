/*
 * @Description: 不滤波
 * @Created Date: 2020-02-09 19:37:49
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:53:34
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class NoFilter : public CloudFilterInterface {
public:
    NoFilter();

    bool Filter(const CloudData::Cloud_Ptr& input_cloud_ptr,
                CloudData::Cloud_Ptr& filtered_cloud_ptr) override;
};
}  // namespace lidar_localization
#endif