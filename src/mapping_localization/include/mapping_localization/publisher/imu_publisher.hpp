/*
 * @Description: 在ros中发布IMU数据
 * @Created Date: 2020-02-05 02:27:30
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define MAPPING_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include "mapping_localization/sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"

namespace mapping_localization {
class IMUPublisher {
public:
    IMUPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, std::string frame_id);
    IMUPublisher() = default;

    void Publish(IMUData imu_data);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
}  // namespace mapping_localization
#endif