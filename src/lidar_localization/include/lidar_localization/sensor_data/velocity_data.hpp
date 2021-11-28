/*
 * @Description: velocity 数据
 * @Created Date: 2019-07-17 18:27:40
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:11:46
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

namespace lidar_localization {
class VelocityData {
public:
    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                         std::deque<VelocityData>& SyncedData,
                         double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};
}  // namespace lidar_localization
#endif