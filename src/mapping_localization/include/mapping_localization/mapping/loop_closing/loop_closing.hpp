/*
 * @Description: 闭环检测算法
 * @Created Date: 2020-02-04 18:52:45
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-01 00:18:14
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_
#define MAPPING_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_

#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <deque>

#include "mapping_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "mapping_localization/models/loop_closure/loop_closure_detector_interface.hpp"
#include "mapping_localization/models/registration/registration_interface.hpp"

#include "mapping_localization/sensor_data/key_frame.hpp"
#include "mapping_localization/sensor_data/loop_pose.hpp"

#include "mapping_localization/tools/tic_toc.hpp"

namespace mapping_localization {
class LoopClosing {
public:
    enum class SearchCriteria { Distance_GNSS = 0, Distance_Odom, ScanContext };

public:
    LoopClosing(const YAML::Node& global_node, const YAML::Node& config_node);

    bool Update(const KeyFrame key_frame, const KeyFrame key_gnss);

    bool HasNewLoopPose();
    LoopPose& GetCurrentLoopPose();

private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user,
                    std::shared_ptr<CloudFilterInterface>& filter_ptr,
                    const YAML::Node& config_node);

    bool CloudRegistration(int key_frame_index);
    bool JointMap(int key_frame_index, CloudData::Cloud_Ptr& map_cloud_ptr, Eigen::Matrix4f& map_pose);
    bool JointScan(CloudData::Cloud_Ptr& scan_cloud_ptr, Eigen::Matrix4f& scan_pose);
    bool Registration(CloudData::Cloud_Ptr& map_cloud_ptr,
                      CloudData::Cloud_Ptr& scan_cloud_ptr,
                      Eigen::Matrix4f& scan_pose,
                      Eigen::Matrix4f& result_pose);

private:
    std::string key_frames_path_ = "";
    int extend_frame_num_ = 3;
    float fitness_score_limit_ = 2.0;

    SearchCriteria search_criteria_ = SearchCriteria::Distance_GNSS;

    std::shared_ptr<LoopClousreDetectorInterface> loop_clousre_detector_ptr_;

    std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::deque<KeyFrame> all_key_frames_;
    std::deque<KeyFrame> all_key_gnss_;

    LoopPose current_loop_pose_;
    bool has_new_loop_pose_ = false;
};
}  // namespace mapping_localization

#endif