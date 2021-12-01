/*
 * @Description:
 * @Created Date: 2021-11-30 22:45:02
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-01 12:57:51
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/loop_closure/distance_detector.hpp"

namespace mapping_localization {
DistanceDetector::DistanceDetector(const YAML::Node& node) {
    loop_step_ = node["loop_step"].as<int>();
    diff_num_ = node["diff_num"].as<int>();
    detect_area_ = node["detect_area"].as<float>();
}

void DistanceDetector::addLatestKeyFrame(const KeyFrame& key_frame, const CloudData::Cloud_Ptr& cloud_data) {
    historical_key_frames.push_back(key_frame);
}
bool DistanceDetector::DetectNearestKeyFrame(int& key_frame_index) {
    static int skip_cnt = 0;
    static int skip_num = loop_step_;
    if (++skip_cnt < skip_num) return false;

    if ((int)historical_key_frames.size() < diff_num_ + 1) return false;

    int key_num = (int)historical_key_frames.size();
    float min_distance = 1000000.0;
    float distance = 0.0;

    KeyFrame history_key_frame;
    KeyFrame current_key_frame = historical_key_frames.back();

    key_frame_index = -1;
    for (int i = 0; i < key_num - 1; ++i) {
        if (key_num - i < diff_num_) break;

        history_key_frame = historical_key_frames.at(i);
        distance = fabs(current_key_frame.pose(0, 3) - history_key_frame.pose(0, 3)) +
                   fabs(current_key_frame.pose(1, 3) - history_key_frame.pose(1, 3)) +
                   fabs(current_key_frame.pose(2, 3) - history_key_frame.pose(2, 3));
        if (distance < min_distance) {
            min_distance = distance;
            key_frame_index = i;
        }
    }

    skip_cnt = 0;
    skip_num = (int)min_distance;
    if (min_distance > detect_area_) {
        skip_num = std::max((int)(min_distance / 2.0), loop_step_);
        return false;
    } else {
        skip_num = loop_step_;
        return true;
    }
}

}  // namespace mapping_localization
