/*
 * @Description:
 * @Created Date: 2021-11-30 12:09:28
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-30 18:46:20
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SUBSCRIBER_TRAJECTORY_SUBSCRIBER_HPP_
#define MAPPING_LOCALIZATION_SUBSCRIBER_TRAJECTORY_SUBSCRIBER_HPP_

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {
class TrajectorySubscriber {
public:
    TrajectorySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    TrajectorySubscriber() = default;
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
