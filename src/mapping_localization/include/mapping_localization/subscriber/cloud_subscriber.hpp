/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Created Date: 2020-02-05 02:27:30
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_CLOUD_SUBSCRIBER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/cloud_data.hpp"

namespace mapping_localization {
class CloudSubscriber {
public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_;
};
}  // namespace mapping_localization

#endif