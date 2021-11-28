/*
 * @Description: voxel filter 模块
 * @Created Date: 2020-02-09 19:37:49
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define MAPPING_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>

#include "mapping_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace mapping_localization {
class VoxelFilter : public CloudFilterInterface {
public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::Cloud_Ptr& input_cloud_ptr, CloudData::Cloud_Ptr& filtered_cloud_ptr) override;

private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

private:
    pcl::VoxelGrid<CloudData::Point> voxel_filter_;
};
}  // namespace mapping_localization
#endif