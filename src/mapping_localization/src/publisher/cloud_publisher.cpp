/*
 * @Description: 通过ros发布点云
 * @Created Date: 2020-02-05 02:27:30
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-25 18:53:18
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/publisher/cloud_publisher.hpp"

#include "glog/logging.h"

namespace mapping_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::Cloud_Ptr& cloud_ptr_input,
                             double time) {
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::Cloud_Ptr& cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

void CloudPublisher::PublishData(CloudData::Cloud_Ptr& cloud_ptr_input,
                                 ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(
        new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}  // namespace mapping_localization