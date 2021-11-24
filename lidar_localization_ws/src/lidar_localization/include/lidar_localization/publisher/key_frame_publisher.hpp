/*
 * @Description: 单个 key frame 信息发布
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:26:18
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <string>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFramePublisher {
public:
    KeyFramePublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string frame_id,
                      int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace lidar_localization
#endif