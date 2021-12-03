/*
 * @Description: 第三方前端接口
 * @Created Date: 2021-12-02 00:04:47
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-03 17:36:43
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_DATA_PRETREAT_EXTERNAL_FRONT_END_ADAPTER_HPP_
#define MAPPING_LOCALIZATION_DATA_PRETREAT_EXTERNAL_FRONT_END_ADAPTER_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <string>

#include "mapping_localization/publisher/odometry_publisher.hpp"
#include "mapping_localization/sensor_data/pose_data.hpp"
#include "mapping_localization/subscriber/odometry_subscriber.hpp"

namespace mapping_localization {

///@brief 用于接收外部里程计的位姿，并和给定点云消息进行同步输出
class ExternalFrontEndAdapter {
private:
public:
    ExternalFrontEndAdapter(std::shared_ptr<OdometrySubscriber>& external_laser_odom_sub_ptr,
                            std::shared_ptr<OdometryPublisher>& synced_external_lasesr_odom_pub_ptr,
                            const std::vector<float>& pose_to_pointcloud);
    ~ExternalFrontEndAdapter();

    bool HasData() const;

    ///
    ///@brief 和指定点云消息时间进行同步，注意：本函数假定外部里程计的位姿估计和点云时间戳一致，因此不会进行插值
    ///@param time 点云时间
    ///@return true 当前里程计消息缓存包含该时间内的位姿估计
    ///
    bool SyncData(double time);

    ///
    ///@brief 发布最旧一帧里程计
    ///
    void PublishData();

private:
    Eigen::Matrix4f pose_to_pointcloud_ = Eigen::Matrix4f::Identity();
    std::deque<PoseData> external_odom_buff_;
    std::shared_ptr<OdometrySubscriber> external_laser_odom_sub_ptr_;
    std::shared_ptr<OdometryPublisher> synced_external_lasesr_odom_pub_ptr_;
};

}  // namespace mapping_localization

#endif