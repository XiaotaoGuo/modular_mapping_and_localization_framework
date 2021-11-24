/*
 * @Description:
 * @Created Date: 2019-07-17 18:17:49
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:11:33
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization {
class CloudData {
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData() : cloud_ptr(new CLOUD()) {}

public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}  // namespace lidar_localization

#endif