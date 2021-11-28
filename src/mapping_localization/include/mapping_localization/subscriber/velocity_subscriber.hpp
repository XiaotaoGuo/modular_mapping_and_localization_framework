/*
 * @Description: 订阅velocity数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "geometry_msgs/TwistStamped.h"
#include "mapping_localization/sensor_data/velocity_data.hpp"

namespace mapping_localization {
class VelocitySubscriber {
public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_;
};
}  // namespace mapping_localization
#endif