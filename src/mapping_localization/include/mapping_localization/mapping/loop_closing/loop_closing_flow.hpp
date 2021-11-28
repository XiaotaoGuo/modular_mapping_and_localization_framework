/*
 * @Description:
 * @Created Date: 2020-02-29 03:32:14
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-28 12:43:22
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define MAPPING_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <ros/ros.h>

#include <deque>
// subscriber
#include "mapping_localization/subscriber/key_frame_subscriber.hpp"
// publisher
#include "mapping_localization/publisher/loop_pose_publisher.hpp"
// loop closing
#include "mapping_localization/mapping/loop_closing/loop_closing.hpp"

namespace mapping_localization {
class LoopClosingFlow {
public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();

private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

private:
    // subscriber
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> key_gnss_buff_;

    KeyFrame current_key_frame_;
    KeyFrame current_key_gnss_;
};
}  // namespace mapping_localization

#endif