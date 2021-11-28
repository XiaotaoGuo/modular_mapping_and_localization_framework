/*
 * @Description: 订阅 key frame 数据
 * @Created Date: 2019-08-19 19:22:17
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {
class KeyFramesSubscriber {
public:
    KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFramesSubscriber() = default;
    void ParseData(std::deque<KeyFrame>& deque_key_frames);

private:
    void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<KeyFrame> new_key_frames_;

    std::mutex buff_mutex_;
};
}  // namespace mapping_localization
#endif