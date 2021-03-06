/*
 * @Description: matching 模块任务管理， 放在类里使代码更清晰
 * @Created Date: 2020-02-10 08:31:22
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_
#define MAPPING_LOCALIZATION_MATCHING_MATCHING_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "mapping_localization/subscriber/cloud_subscriber.hpp"
#include "mapping_localization/subscriber/odometry_subscriber.hpp"
// publisher
#include "mapping_localization/publisher/cloud_publisher.hpp"
#include "mapping_localization/publisher/odometry_publisher.hpp"
#include "mapping_localization/publisher/tf_broadcaster.hpp"
// matching
#include "mapping_localization/matching/matching.hpp"

namespace mapping_localization {
class MatchingFlow {
public:
    MatchingFlow(ros::NodeHandle& nh);

    bool Run();

private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();
    bool PublishData();

private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;
    // matching
    std::shared_ptr<Matching> matching_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_data_buff_;

    CloudData current_cloud_data_;
    PoseData current_gnss_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace mapping_localization

#endif