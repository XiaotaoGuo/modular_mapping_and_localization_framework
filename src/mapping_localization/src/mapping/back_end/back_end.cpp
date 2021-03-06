/*
 * @Description: back end 具体实现
 * @Created Date: 2020-02-28 01:02:51
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-15 12:52:25
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/back_end/back_end.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

#include "glog/logging.h"
#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/tools/file_manager.hpp"

namespace mapping_localization {
BackEnd::BackEnd(const YAML::Node& global_node, const YAML::Node& config_node) {
    LOG(INFO) << "-----------------后端初始化-------------------";
    InitParam(config_node);
    InitGraphOptimizer(config_node);
    InitDataPath(global_node);
}

bool BackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();

    return true;
}

bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
    std::string graph_optimizer_type;
    std::vector<std::string> options{"g2o", "ceres", "gtsam"};
    if (!try_load_param(config_node, "graph_optimizer_type", graph_optimizer_type, std::string("g2o"), options)) {
        LOG(ERROR) << "没有找到与 " << graph_optimizer_type << " 对应的图优化模式,请检查配置文件";
        return false;
    } else {
        if (graph_optimizer_type == "g2o") {
            graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>(config_node[graph_optimizer_type]);
        } else if (graph_optimizer_type == "ceres") {
            graph_optimizer_ptr_ = std::make_shared<CeresGraphOptimizer>(config_node[graph_optimizer_type]);
        } else if (graph_optimizer_type == "gtsam") {
            graph_optimizer_ptr_ = std::make_shared<GTSamGraphOptimizer>(config_node[graph_optimizer_type]);
        }
    }

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame = config_node["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["optimize_step_with_loop"].as<int>();

    std::vector<double> vec;
    try_load_param(config_node["noise_model"]["odom"], "translation", vec, std::vector<double>{0.5, 0.5, 0.5});
    graph_optimizer_config_.odom_translation_noise = Eigen::Map<Eigen::Vector3d>(vec.data());
    try_load_param(config_node["noise_model"]["odom"], "rotation", vec, std::vector<double>{0.001, 0.001, 0.001});
    graph_optimizer_config_.odom_rotation_noise = Eigen::Map<Eigen::Vector3d>(vec.data());

    try_load_param(config_node["noise_model"]["close_loop"], "translation", vec, std::vector<double>{0.3, 0.3, 0.3});
    graph_optimizer_config_.close_loop_translation_noise = Eigen::Map<Eigen::Vector3d>(vec.data());
    try_load_param(config_node["noise_model"]["close_loop"], "rotation", vec, std::vector<double>{0.001, 0.001, 0.001});
    graph_optimizer_config_.close_loop_rotation_noise = Eigen::Map<Eigen::Vector3d>(vec.data());

    try_load_param(config_node["noise_model"]["gnss"], "translation", vec, std::vector<double>{2.0, 2.0, 2.0});
    graph_optimizer_config_.gnss_translation_noise = Eigen::Map<Eigen::Vector3d>(vec.data());

    return true;
}

bool BackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data")) return false;

    key_frames_path_ = data_path + "/slam_data/key_frames";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!FileManager::InitDirectory(key_frames_path_, "关键帧点云")) return false;
    if (!FileManager::InitDirectory(trajectory_path_, "轨迹文件")) return false;

    if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt")) return false;
    if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt")) return false;

    return true;
}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose) {
    ResetParam();

    if (MaybeNewKeyFrame(cloud_data, laser_odom, gnss_pose)) {
        SavePose(ground_truth_ofs_, gnss_pose.pose);
        SavePose(laser_odom_ofs_, laser_odom.pose);
        AddNodeAndEdge(gnss_pose);

        if (MaybeOptimized()) {
            SaveOptimizedPose();
        }
    }

    return true;
}

bool BackEnd::InsertLoopPose(const LoopPose& loop_pose) {
    if (!graph_optimizer_config_.use_loop_close) return false;

    Eigen::Isometry3d isometry;
    isometry.matrix() = loop_pose.pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Edge(loop_pose.index0,
                                     loop_pose.index1,
                                     isometry,
                                     graph_optimizer_config_.close_loop_translation_noise,
                                     graph_optimizer_config_.close_loop_rotation_noise);

    new_loop_cnt_++;
    // LOG(INFO) << "插入闭环：" << loop_pose.index0 << "," << loop_pose.index1;

    return true;
}

void BackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool BackEnd::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);

            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_odom) {
    static Eigen::Matrix4f last_key_pose = laser_odom.pose;
    if (key_frames_deque_.size() == 0) {
        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    }

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(laser_odom.pose(0, 3) - last_key_pose(0, 3)) + fabs(laser_odom.pose(1, 3) - last_key_pose(1, 3)) +
            fabs(laser_odom.pose(2, 3) - last_key_pose(2, 3)) >
        key_frame_distance_) {
        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    }

    if (has_new_key_frame_) {
        // 把关键帧点云存储到硬盘里
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);
        current_key_frame_ = key_frame;

        current_key_gnss_.time = gnss_odom.time;
        current_key_gnss_.index = key_frame.index;
        current_key_gnss_.pose = gnss_odom.pose;
    }

    return has_new_key_frame_;
}

bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    Eigen::Isometry3d isometry;
    // 添加关键帧节点
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    if (graph_optimizer_ptr_->GetNodeNum() == 0)
        graph_optimizer_ptr_->AddSe3Node(isometry, true);
    else
        graph_optimizer_ptr_->AddSe3Node(isometry, false);
    new_key_frame_cnt_++;

    // 添加激光里程计对应的边
    static KeyFrame last_key_frame = current_key_frame_;
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if (node_num > 1) {
        Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddSe3Edge(node_num - 2,
                                         node_num - 1,
                                         isometry,
                                         graph_optimizer_config_.odom_translation_noise,
                                         graph_optimizer_config_.odom_rotation_noise);
    }
    last_key_frame = current_key_frame_;

    // 添加gnss位置对应的先验边
    if (graph_optimizer_config_.use_gnss) {
        Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0, 3)),
                            static_cast<double>(gnss_data.pose(1, 3)),
                            static_cast<double>(gnss_data.pose(2, 3)));
        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num - 1, xyz, graph_optimizer_config_.gnss_translation_noise);
        new_gnss_cnt_++;
    }

    return true;
}

bool BackEnd::MaybeOptimized() {
    bool need_optimize = false;

    if (new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss) need_optimize = true;
    if (new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop) need_optimize = true;
    if (new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame) need_optimize = true;

    if (!need_optimize) return false;

    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    new_key_frame_cnt_ = 0;

    if (graph_optimizer_ptr_->Optimize()) has_new_optimized_ = true;

    return true;
}

bool BackEnd::SaveOptimizedPose() {
    if (graph_optimizer_ptr_->GetNodeNum() == 0) return false;

    if (!FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt")) return false;

    graph_optimizer_ptr_->GetOptimizedPose(optimized_pose_);

    for (size_t i = 0; i < optimized_pose_.size(); ++i) {
        SavePose(optimized_pose_ofs_, optimized_pose_.at(i));
    }

    pose_to_optimize_ = optimized_pose_.back() * key_frames_deque_.back().pose.inverse();

    return true;
}

bool BackEnd::ForceOptimize() {
    if (graph_optimizer_ptr_->Optimize()) has_new_optimized_ = true;

    SaveOptimizedPose();

    return has_new_optimized_;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
    KeyFrame key_frame;
    for (size_t i = 0; i < optimized_pose_.size(); ++i) {
        key_frame.pose = optimized_pose_.at(i);
        key_frame.index = (unsigned int)i;
        key_frames_deque.push_back(key_frame);
    }
}

bool BackEnd::HasNewKeyFrame() { return has_new_key_frame_; }

bool BackEnd::HasNewOptimized() { return has_new_optimized_; }

void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame) { key_frame = current_key_frame_; }

void BackEnd::GetLatestKeyGNSS(KeyFrame& key_frame) { key_frame = current_key_gnss_; }

void BackEnd::GetLatestCorrectedKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
    key_frame.pose = pose_to_optimize_ * current_key_frame_.pose;
}
}  // namespace mapping_localization