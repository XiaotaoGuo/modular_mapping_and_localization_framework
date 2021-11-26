/*
 * @Description:
 * @Created Date: 2019-07-17 18:17:49
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:52:59
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization {
class CloudData {
public:
    using Point = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<Point>;
    using Cloud_Ptr = Cloud::Ptr;

public:
    CloudData() : cloud_ptr(new Cloud()) {}

public:
    double time = 0.0;
    Cloud_Ptr cloud_ptr;
};
}  // namespace lidar_localization

#endif