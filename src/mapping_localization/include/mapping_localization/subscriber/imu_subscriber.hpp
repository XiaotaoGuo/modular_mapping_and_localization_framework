/*
 * @Description: 订阅imu数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/imu_data.hpp"
#include "sensor_msgs/Imu.h"

namespace mapping_localization {
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
}  // namespace mapping_localization
#endif