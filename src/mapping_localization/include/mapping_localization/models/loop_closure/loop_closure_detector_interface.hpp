/*
 * @Description:
 * @Created Date: 2021-11-30 21:58:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-30 22:25:45
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_LOOP_CLOSURE_DETECTOR_INTERFACE_HPP_
#define MAPPING_LOCALIZATION_MODELS_LOOP_CLOSURE_LOOP_CLOSURE_DETECTOR_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "mapping_localization/sensor_data/cloud_data.hpp"
#include "mapping_localization/sensor_data/key_frame.hpp"
#include "mapping_localization/sensor_data/pose_data.hpp"

namespace mapping_localization {

class LoopClousreDetectorInterface {
public:
    virtual ~LoopClousreDetectorInterface() = default;

    virtual void addLatestKeyFrame(const KeyFrame& key_frame, const CloudData::Cloud_Ptr& cloud_data) = 0;
    virtual bool DetectNearestKeyFrame(int& key_frame_index) = 0;
};
}  // namespace mapping_localization
#endif