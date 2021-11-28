/*
 * @Description: 从点云中截取一个立方体部分
 * @Created Date: 2020-03-04 20:09:37
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_
#define MAPPING_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_HPP_

#include <pcl/filters/crop_box.h>

#include "mapping_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace mapping_localization {
class BoxFilter : public CloudFilterInterface {
public:
    BoxFilter(YAML::Node node);
    BoxFilter() = default;

    bool Filter(const CloudData::Cloud_Ptr& input_cloud_ptr, CloudData::Cloud_Ptr& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

private:
    void CalculateEdge();

private:
    pcl::CropBox<CloudData::Point> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}  // namespace mapping_localization

#endif