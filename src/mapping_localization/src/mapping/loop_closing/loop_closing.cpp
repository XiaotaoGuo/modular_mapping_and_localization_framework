/*
 * @Description: 闭环检测算法
 * @Created Date: 2020-02-04 18:53:06
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-01 13:10:02
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/mapping/loop_closing/loop_closing.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>

#include "glog/logging.h"
#include "mapping_localization/global_defination/global_defination.h"

#include "mapping_localization/models/cloud_filter/no_filter.hpp"
#include "mapping_localization/models/cloud_filter/voxel_filter.hpp"

#include "mapping_localization/models/registration/icp_registration.hpp"
#include "mapping_localization/models/registration/ndt_registration.hpp"

#include "mapping_localization/models/loop_closure/distance_detector.hpp"
#include "mapping_localization/models/loop_closure/scan_context_detector.hpp"

#include "mapping_localization/tools/print_info.hpp"

namespace mapping_localization {
LoopClosing::LoopClosing(const YAML::Node& global_node, const YAML::Node& config_node) {
    std::cout << "-----------------闭环检测初始化-------------------" << std::endl;
    InitParam(config_node);
    InitDataPath(global_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("map", map_filter_ptr_, config_node);
    InitFilter("scan", scan_filter_ptr_, config_node);
}

bool LoopClosing::InitParam(const YAML::Node& config_node) {
    extend_frame_num_ = config_node["extend_frame_num"].as<int>();
    fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

    std::string search_criteria = config_node["search_criteria"].as<std::string>();
    if (search_criteria == "scan_context") {
        search_criteria_ = SearchCriteria::ScanContext;
        loop_clousre_detector_ptr_ = std::make_shared<ScanContextDetector>(config_node[search_criteria]);
    } else if (search_criteria == "distance_gnss") {
        search_criteria_ = SearchCriteria::Distance_GNSS;
        loop_clousre_detector_ptr_ = std::make_shared<DistanceDetector>(config_node[search_criteria]);
    } else if (search_criteria == "distance_odom") {
        search_criteria_ = SearchCriteria::Distance_Odom;
        loop_clousre_detector_ptr_ = std::make_shared<DistanceDetector>(config_node[search_criteria]);
    } else {
        std::cout << "没有和 " << search_criteria << " 对应的回环检测检测。 默认使用 GNSS 位置作为回环检测方式,\n";
        search_criteria_ = SearchCriteria::Distance_GNSS;
        loop_clousre_detector_ptr_ = std::make_shared<ScanContextDetector>(config_node["scan_context"]);
    }

    return true;
}

bool LoopClosing::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";

    return true;
}

bool LoopClosing::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,
                                   const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "闭环点云匹配方式为：" << registration_method;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool LoopClosing::InitFilter(std::string filter_user,
                             std::shared_ptr<CloudFilterInterface>& filter_ptr,
                             const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << "闭环的" << filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool LoopClosing::Update(const KeyFrame key_frame, const KeyFrame key_gnss) {
    has_new_loop_pose_ = false;

    all_key_frames_.push_back(key_frame);
    all_key_gnss_.push_back(key_gnss);

    // 从硬盘中读取关键帧点云
    CloudData::Cloud_Ptr scan_cloud_ptr(new CloudData::Cloud());
    std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
    pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

    static Timer loop_closure_timer("Loop Closure");
    loop_closure_timer.tic();
    int key_frame_index = 0;
    if (search_criteria_ == SearchCriteria::Distance_GNSS) {
        // 基于距离搜索距离较近的关键帧
        loop_clousre_detector_ptr_->addLatestKeyFrame(key_gnss, scan_cloud_ptr);
    } else if (search_criteria_ == SearchCriteria::Distance_Odom) {
        // 基于里程计位置估计搜索较近的关键帧
        loop_clousre_detector_ptr_->addLatestKeyFrame(key_frame, scan_cloud_ptr);
    } else if (search_criteria_ == SearchCriteria::ScanContext) {
        // 将下采样后的点云传入 sc manager 构建并存储 scan context
        loop_clousre_detector_ptr_->addLatestKeyFrame(key_frame, scan_cloud_ptr);
    }

    has_new_loop_pose_ = loop_clousre_detector_ptr_->DetectNearestKeyFrame(key_frame_index);
    loop_closure_timer.toc();

    if (!has_new_loop_pose_) return false;

    // 初始化回环信息（此时置信度设为 false）
    current_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;
    current_loop_pose_.index1 = all_key_frames_.back().index;
    current_loop_pose_.time = all_key_frames_.back().time;
    current_loop_pose_.confidence = false;

    std::cout << "检测到闭环 "
              << ": 帧" << current_loop_pose_.index0 << "------>"
              << "帧" << current_loop_pose_.index1 << std::endl;

    if (key_frame_index < extend_frame_num_) {
        std::cout << "匹配失败！\n";
        return false;
    }

    if (CloudRegistration(key_frame_index)) {
        current_loop_pose_.confidence = true;
    }

    LOG(INFO) << loop_closure_timer;

    return true;
}

bool LoopClosing::CloudRegistration(int key_frame_index) {
    // 生成地图
    CloudData::Cloud_Ptr map_cloud_ptr(new CloudData::Cloud());
    Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
    JointMap(key_frame_index, map_cloud_ptr, map_pose);

    // 生成当前scan
    CloudData::Cloud_Ptr scan_cloud_ptr(new CloudData::Cloud());
    Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
    JointScan(scan_cloud_ptr, scan_pose);

    // 匹配
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Registration(map_cloud_ptr, scan_cloud_ptr, scan_pose, result_pose);

    // 计算相对位姿
    current_loop_pose_.pose = map_pose.inverse() * result_pose;

    // 判断是否有效
    if (registration_ptr_->GetFitnessScore() > fitness_score_limit_) {
        std::cout << "匹配失败！\n";
        return false;
    }

    static int loop_close_cnt = 0;
    loop_close_cnt++;

    std::cout << "匹配成功，fitness score: " << registration_ptr_->GetFitnessScore() << " 回环数量" << loop_close_cnt
              << std::endl
              << std::endl;

    // std::cout << "相对位姿 x y z roll pitch yaw:";
    // PrintInfo::PrintPose("", current_loop_pose_.pose);

    return true;
}

bool LoopClosing::JointMap(int key_frame_index, CloudData::Cloud_Ptr& map_cloud_ptr, Eigen::Matrix4f& map_pose) {
    map_pose = all_key_gnss_.at(key_frame_index).pose;

    // 合成地图
    Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(key_frame_index).pose.inverse();

    for (int i = key_frame_index - extend_frame_num_; i < key_frame_index + extend_frame_num_; ++i) {
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";

        CloudData::Cloud_Ptr cloud_ptr(new CloudData::Cloud());
        pcl::io::loadPCDFile(file_path, *cloud_ptr);

        Eigen::Matrix4f cloud_pose = pose_to_gnss * all_key_frames_.at(i).pose;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);

        *map_cloud_ptr += *cloud_ptr;
    }
    map_filter_ptr_->Filter(map_cloud_ptr, map_cloud_ptr);
    return true;
}

bool LoopClosing::JointScan(CloudData::Cloud_Ptr& scan_cloud_ptr, Eigen::Matrix4f& scan_pose) {
    scan_pose = all_key_gnss_.back().pose;

    std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
    pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

    return true;
}

bool LoopClosing::Registration(CloudData::Cloud_Ptr& map_cloud_ptr,
                               CloudData::Cloud_Ptr& scan_cloud_ptr,
                               Eigen::Matrix4f& scan_pose,
                               Eigen::Matrix4f& result_pose) {
    // 点云匹配
    CloudData::Cloud_Ptr result_cloud_ptr(new CloudData::Cloud());
    registration_ptr_->SetInputTarget(map_cloud_ptr);
    registration_ptr_->ScanMatch(scan_cloud_ptr, scan_pose, result_cloud_ptr, result_pose);

    return true;
}

bool LoopClosing::HasNewLoopPose() { return has_new_loop_pose_; }

LoopPose& LoopClosing::GetCurrentLoopPose() { return current_loop_pose_; }
}  // namespace mapping_localization