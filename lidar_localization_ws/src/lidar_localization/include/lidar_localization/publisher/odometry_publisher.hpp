/*
 * @Description: odometry 信息发布
 * @Created Date: 2020-02-06 21:05:47
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:27:22
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <string>

namespace lidar_localization {
class OdometryPublisher {
public:
    OdometryPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);

    bool HasSubscribers();

private:
    void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};
}  // namespace lidar_localization
#endif