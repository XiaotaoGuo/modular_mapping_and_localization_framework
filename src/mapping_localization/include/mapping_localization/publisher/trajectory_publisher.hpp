/*
 * @Description:
 * @Created Date: 2021-11-30 11:03:52
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-30 23:47:57
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_PUBLISHER_TRAJECTORY_PUBLISHER_HPP_
#define MAPPING_LOCALIZATION_PUBLISHER_TRAJECTORY_PUBLISHER_HPP_

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <deque>
#include <string>

#include "mapping_localization/sensor_data/key_frame.hpp"

namespace mapping_localization {

class TrajectoryPublisher {
public:
    TrajectoryPublisher(ros::NodeHandle& nh,
                        const std::string& topic_name,
                        const std::string& frame_id,
                        int buff_size,
                        bool has_edge_info = false);

    ///@brief 在原有轨迹基础上增加新的位姿，并发布
    ///@param key_frame 新的关键帧
    void Publish(const KeyFrame& key_frame);

    ///@brief 重置所有关键帧为输入关键帧序列并更新发布
    ///@param key_frames 新的关键帧序列
    void Publish(const std::deque<KeyFrame>& key_frames);

    ///
    ///@brief 在轨迹中两个点连线
    ///@param index1
    ///@param index2
    ///
    bool AddEdge(unsigned int index1, unsigned int index2, bool is_true);

    bool HasSubscribers();

private:
    geometry_msgs::PoseStamped convertKeyFrameToPose(const KeyFrame& key_frame, const std::string& frame_id);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Path path_;

    bool has_edge_info_ = false;

    ros::Publisher true_edge_publisher_;
    ros::Publisher false_edge_publisher_;
    visualization_msgs::Marker true_edge_list;
    visualization_msgs::Marker false_edge_list;
};

}  // namespace mapping_localization

#endif