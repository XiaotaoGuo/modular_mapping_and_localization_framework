/*
 * @Description: 基于 ScanContext 进行回环检测
 * @Created Date: 2021-11-30 22:13:00
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-30 22:45:25
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_SCAN_CONTEXT_DETECTOR_HPP_
#define MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_SCAN_CONTEXT_DETECTOR_HPP_

#include "mapping_localization/mapping/loop_closing/ScanContext/Scancontext.h"

#include "mapping_localization/models/loop_closure/loop_closure_detector_interface.hpp"

namespace mapping_localization {
class ScanContextDetector : public LoopClousreDetectorInterface {
public:
    ScanContextDetector(const YAML::Node& node);
    void addLatestKeyFrame(const KeyFrame& key_frame, const CloudData::Cloud_Ptr& cloud_data) override;
    bool DetectNearestKeyFrame(int& key_frame_index) override;

private:
    SCManager sc_manager_;
};

}  // namespace mapping_localization
#endif