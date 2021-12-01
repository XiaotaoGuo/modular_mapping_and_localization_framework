/*
 * @Description:
 * @Created Date: 2019-07-17 18:27:40
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-29 19:40:55
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <deque>

namespace mapping_localization {
class IMUData {
public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    class Orientation {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

    public:
        void Normlize() {
            double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();

    ///
    ///@brief 清除未同步缓存中早于指定时间的数据，并取两个离指定时间最近的数据进行线性插值
    ///
    ///@param UnsyncedData - 未经过时间同步的消息队列
    ///@param SyncedData - 经过时间同步的消息队列（最后一个数据是指定时间的插值 IMU 数据）
    ///@param sync_time - 指定同步时间
    ///@return true - 同步成功，false - 同步失败
    ///
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
}  // namespace mapping_localization
#endif