/*
 * @Description: 订阅imu数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:28:52
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "lidar_localization/sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"

namespace lidar_localization {
class IMUSubscriber {
public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData>& deque_imu_data);

private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_;

    std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif