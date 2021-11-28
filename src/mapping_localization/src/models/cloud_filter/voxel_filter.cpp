/*
 * @Description: voxel filter 模块实现
 * @Created Date: 2020-02-09 19:53:20
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:53:20
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace mapping_localization {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();

    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x,
                         float leaf_size_y,
                         float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x,
                                 float leaf_size_y,
                                 float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter 的参数为：" << std::endl
              << leaf_size_x << ", " << leaf_size_y << ", " << leaf_size_z
              << std::endl
              << std::endl;

    return true;
}

bool VoxelFilter::Filter(const CloudData::Cloud_Ptr& input_cloud_ptr,
                         CloudData::Cloud_Ptr& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
}  // namespace mapping_localization