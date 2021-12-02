/*
 * @Description:
 * @Created Date: 2021-12-02 00:46:57
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-02 11:36:32
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/data_pretreat/external_front_end_adapter.hpp"

namespace mapping_localization {

ExternalFrontEndAdapter::ExternalFrontEndAdapter(
    std::shared_ptr<OdometrySubscriber>& external_laser_odom_sub_ptr,
    std::shared_ptr<OdometryPublisher>& synced_external_lasesr_odom_pub_ptr)
    : external_laser_odom_sub_ptr_(external_laser_odom_sub_ptr),
      synced_external_lasesr_odom_pub_ptr_(synced_external_lasesr_odom_pub_ptr) {}

bool ExternalFrontEndAdapter::SyncData(double time) {
    external_laser_odom_sub_ptr_->ParseData(external_odom_buff_);

    while (!external_odom_buff_.empty() && external_odom_buff_.front().time < time) {
        external_odom_buff_.pop_front();
    }

    if (external_odom_buff_.empty() || external_odom_buff_.front().time != time) {
        return false;
    }

    return true;
}

bool ExternalFrontEndAdapter::HasData() const { return external_odom_buff_.size(); }

void ExternalFrontEndAdapter::PublishData() {
    PoseData pose = external_odom_buff_.front();
    external_odom_buff_.pop_front();

    synced_external_lasesr_odom_pub_ptr_->Publish(pose.pose, pose.time);
}

ExternalFrontEndAdapter::~ExternalFrontEndAdapter() {}
}  // namespace mapping_localization
