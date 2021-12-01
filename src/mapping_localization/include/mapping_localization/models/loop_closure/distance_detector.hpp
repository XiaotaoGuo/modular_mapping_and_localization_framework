/*
 * @Description: 基于位置比较进行回环检测
 * @Created Date: 2021-11-30 22:40:53
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-01 12:57:54
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_DISTANCE_DETECTOR_HPP_
#define MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_DISTANCE_DETECTOR_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <deque>

#include <pcl/kdtree/kdtree_flann.h>

#include "mapping_localization/sensor_data/cloud_data.hpp"
#include "mapping_localization/sensor_data/key_frame.hpp"
#include "mapping_localization/sensor_data/pose_data.hpp"

#include "mapping_localization/models/loop_closure/loop_closure_detector_interface.hpp"

namespace mapping_localization {

class DistanceDetector : public LoopClousreDetectorInterface {
public:
    DistanceDetector(const YAML::Node& node);

    void addLatestKeyFrame(const KeyFrame& key_frame, const CloudData::Cloud_Ptr& cloud_data) override;
    bool DetectNearestKeyFrame(int& key_frame_index) override;

private:
    int loop_step_ = 5;
    int diff_num_ = 10.0;
    int detect_area_ = 10.0;
    std::deque<KeyFrame> historical_key_frames;
};
}  // namespace mapping_localization

#endif