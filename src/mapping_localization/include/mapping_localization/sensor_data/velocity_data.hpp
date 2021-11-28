/*
 * @Description: velocity 数据
 * @Created Date: 2019-07-17 18:27:40
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <Eigen/Dense>
#include <deque>

namespace mapping_localization {
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
}  // namespace mapping_localization
#endif