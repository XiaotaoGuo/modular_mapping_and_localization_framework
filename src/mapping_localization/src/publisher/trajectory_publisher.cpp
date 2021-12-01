/*
 * @Description:
 * @Created Date: 2021-11-30 11:09:55
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-01 00:14:15
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/publisher/trajectory_publisher.hpp"

#include <Eigen/Dense>

namespace mapping_localization {
TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle& nh,
                                         const std::string& topic_name,
                                         const std::string& frame_id,
                                         int buff_size,
                                         bool has_edge_info)
    : nh_(nh), has_edge_info_(has_edge_info) {
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    path_.header.frame_id = frame_id;

    if (has_edge_info_) {
        true_edge_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name + "_true_loop", buff_size);
        false_edge_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name + "_false_loop", buff_size);

        true_edge_list.header.frame_id = frame_id;
        true_edge_list.ns = "loop edge";
        true_edge_list.id = 0;
        true_edge_list.type = visualization_msgs::Marker::LINE_LIST;
        true_edge_list.scale.x = 0.3;
        true_edge_list.color.r = 1.0;
        true_edge_list.color.a = 1.0;

        false_edge_list = true_edge_list;
        false_edge_list.id = 1;
        false_edge_list.color.g = 1.0;
    }
}

void TrajectoryPublisher::Publish(const KeyFrame& key_frame) {
    path_.header.stamp = ros::Time::now();  // TODO: change to latest key frame??
    path_.poses.push_back(convertKeyFrameToPose(key_frame, path_.header.frame_id));

    true_edge_list.header.stamp = ros::Time::now();
    false_edge_list.header.stamp = ros::Time::now();

    if (!true_edge_list.points.empty()) {
        true_edge_publisher_.publish(true_edge_list);
    }
    if (!false_edge_list.points.empty()) {
        false_edge_publisher_.publish(false_edge_list);
    }

    publisher_.publish(path_);
}

void TrajectoryPublisher::Publish(const std::deque<KeyFrame>& key_frames) {
    path_.header.stamp = ros::Time::now();  // TODO: change to latest key frame??

    path_.poses.clear();
    for (const KeyFrame& key_frame : key_frames) {
        path_.poses.push_back(convertKeyFrameToPose(key_frame, path_.header.frame_id));
    }

    publisher_.publish(path_);
}

bool TrajectoryPublisher::AddEdge(unsigned int index1, unsigned int index2, bool is_true) {
    if (index1 < 0 || index1 >= path_.poses.size() || index2 < 0 || index2 >= path_.poses.size()) {
        return false;
    }

    if (is_true) {
        true_edge_list.points.push_back(path_.poses[index1].pose.position);
        true_edge_list.points.push_back(path_.poses[index2].pose.position);
    } else {
        false_edge_list.points.push_back(path_.poses[index1].pose.position);
        false_edge_list.points.push_back(path_.poses[index2].pose.position);
    }

    return true;
}

geometry_msgs::PoseStamped TrajectoryPublisher::convertKeyFrameToPose(const KeyFrame& key_frame,
                                                                      const std::string& frame_id) {
    geometry_msgs::PoseStamped pose_stamped;
    ros::Time ros_time((float)key_frame.time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id;

    pose_stamped.header.seq = key_frame.index;

    pose_stamped.pose.position.x = key_frame.pose(0, 3);
    pose_stamped.pose.position.y = key_frame.pose(1, 3);
    pose_stamped.pose.position.z = key_frame.pose(2, 3);

    Eigen::Quaternionf q = key_frame.GetQuaternion();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    return pose_stamped;
}
bool TrajectoryPublisher::HasSubscribers() { return publisher_.getNumSubscribers() != 0; }
}  // namespace mapping_localization
