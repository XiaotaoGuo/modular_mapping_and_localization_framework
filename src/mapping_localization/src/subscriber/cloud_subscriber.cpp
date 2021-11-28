/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Created Date: 2020-02-05 02:27:30
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:41:10
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace mapping_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh,
                                 std::string topic_name,
                                 size_t buff_size)
    : nh_(nh) {
    subscriber_ =
        nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    new_cloud_data_.push_back(cloud_data);
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(
            cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    buff_mutex_.unlock();
}
}  // namespace mapping_localization