/*
 * @Description: 单个 key frame 信息发布
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define MAPPING_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <string>

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {
class KeyFramePublisher {
public:
    KeyFramePublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}  // namespace mapping_localization
#endif