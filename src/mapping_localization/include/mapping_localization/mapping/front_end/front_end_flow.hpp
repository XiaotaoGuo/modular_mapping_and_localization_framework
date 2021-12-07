/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef MAPPING_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MAPPING_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "mapping_localization/mapping/front_end/front_end.hpp"

#include "mapping_localization/publisher/cloud_publisher.hpp"
#include "mapping_localization/publisher/odometry_publisher.hpp"
#include "mapping_localization/publisher/tf_broadcaster.hpp"

#include "mapping_localization/subscriber/cloud_subscriber.hpp"

namespace mapping_localization {
class FrontEndFlow {
public:
    FrontEndFlow(ros::NodeHandle& nh);

    bool Run();

private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;

    std::shared_ptr<CloudPublisher> prcessed_cloud_pub_ptr_;

    std::shared_ptr<TFBroadCaster> laser_odom_tf_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace mapping_localization

#endif
