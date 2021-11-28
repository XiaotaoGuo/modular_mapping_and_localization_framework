/*
 * @Description:
 * @Created Date: 2019-03-31 12:58:10
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/gnss_data.hpp"
#include "sensor_msgs/NavSatFix.h"

namespace mapping_localization {
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
}  // namespace mapping_localization
#endif