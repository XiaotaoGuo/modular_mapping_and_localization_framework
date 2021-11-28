/*
 * @Description:
 * @Created Date: 2019-03-31 12:58:10
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:28:35
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "lidar_localization/sensor_data/gnss_data.hpp"
#include "sensor_msgs/NavSatFix.h"

namespace lidar_localization {
class GNSSSubscriber {
public:
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;

    std::mutex buff_mutex_;
};
}  // namespace lidar_localization
#endif