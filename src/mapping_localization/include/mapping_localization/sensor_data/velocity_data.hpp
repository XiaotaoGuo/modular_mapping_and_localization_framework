/*
 * @Description: velocity 数据
 * @Created Date: 2019-07-17 18:27:40
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-29 19:41:54
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
    ///
    ///@brief 清除未同步缓存中早于指定时间的数据，并取两个离指定时间最近的数据进行线性插值
    ///
    ///@param UnsyncedData - 未经过时间同步的消息队列
    ///@param SyncedData - 经过时间同步的消息队列（最后一个数据是指定时间的插值速度数据）
    ///@param sync_time - 指定同步时间
    ///@return true - 同步成功，false - 同步失败
    ///
    static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                         std::deque<VelocityData>& SyncedData,
                         double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
};
}  // namespace mapping_localization
#endif