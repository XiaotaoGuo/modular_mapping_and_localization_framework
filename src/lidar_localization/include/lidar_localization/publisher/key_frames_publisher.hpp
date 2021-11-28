/*
 * @Description: key frames 信息发布
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:26:40
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include <string>

#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
class KeyFramesPublisher {
public:
    KeyFramesPublisher(ros::NodeHandle& nh,
                       std::string topic_name,
                       std::string frame_id,
                       int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace lidar_localization
#endif