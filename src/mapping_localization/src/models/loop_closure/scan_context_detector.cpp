/*
 * @Description:
 * @Created Date: 2021-11-30 22:18:08
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-01 14:13:44
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/models/loop_closure/scan_context_detector.hpp"

namespace mapping_localization {

ScanContextDetector::ScanContextDetector(const YAML::Node& node) {
    unsigned int num_exclude_recent = node["num_exclude_recent"].as<unsigned int>();
    double search_ratio = node["search_ratio"].as<double>();
    double sc_dist_thres = node["sc_dist_thres"].as<double>();

    std::cout << "Scan context 使用的参数为："
              << "num_exclude_recent: " << num_exclude_recent << ", search_ratio: " << search_ratio
              << ", sc_dist_thres: " << sc_dist_thres << std::endl;
    sc_manager_.setParameters(num_exclude_recent, search_ratio, sc_dist_thres);
}

void ScanContextDetector::addLatestKeyFrame(const KeyFrame& key_frame, const CloudData::Cloud_Ptr& cloud_data) {
    sc_manager_.makeAndSaveScancontextAndKeys(*cloud_data);
}

bool ScanContextDetector::DetectNearestKeyFrame(int& key_frame_index) {
    auto detect_result = sc_manager_.detectLoopClosureID();  // first: nn index, second: yaw diff
    if (detect_result.first == -1) return false;
    key_frame_index = detect_result.first;
    return true;
}
}  // namespace mapping_localization