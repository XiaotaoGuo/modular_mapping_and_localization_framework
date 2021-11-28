/*
 * @Description: key frames 信息发布
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define MAPPING_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include <string>

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {
class KeyFramesPublisher {
public:
    KeyFramesPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace mapping_localization
#endif