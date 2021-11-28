/*
 * @Description:
 * @Created Date: 2021-11-25 19:07:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-11-27 20:30:08
 * @Modified By: Xiaotao Guo
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {

class ICPRegistration : public RegistrationInterface {
public:
    enum class MatchMethod { PCL = 0, SVD, GaussianNewton };

    struct Param {
        float max_correspodence_dist = 0.0;
        float trans_eps = 0.0;
        float fitness_eps = 0.0;
        int max_iter = 0;
    };

public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(const std::string& match_method,
                    float max_correspodence_dist,
                    float trans_eps,
                    float fitness_eps,
                    int max_iter);

    ///
    ///@brief set input target pointcloud
    ///@param input_target
    ///
    bool SetInputTarget(const CloudData::Cloud_Ptr& input_target) override;

    ///
    ///@brief perform a scan match between input source and target
    ///
    ///@param input_source input point cloud
    ///@param predict_pose initial pose
    ///@param result_cloud_ptr final estimated source point cloud in target pointcloud frame
    ///@param result_pose estimated pose
    ///@return true
    ///@return false
    ///
    bool ScanMatch(const CloudData::Cloud_Ptr& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::Cloud_Ptr& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;

private:
    bool SetRegistraionParam(float max_correspodence_dist, float trans_eps, float fitness_eps, int max_iter);

    void InitializeMatchMethod(const std::string& match_method);

    ///
    ///@brief use svd to esitimate relative transformation
    ///
    ///@param input_source input point cloud
    ///@param predict_pose initial pose
    ///@param result_pose  estimated pose
    ///@return true (TODO)
    ///
    bool MatchWithSVD(const CloudData::Cloud_Ptr& input_source,
                      const Eigen::Matrix4f& predict_pose,
                      Eigen::Matrix4f& result_pose);

    ///
    ///@brief use gaussian newton method to esitimate relative transformation
    ///
    ///@param input_source input point cloud
    ///@param predict_pose initial pose
    ///@param result_pose  estimated pose
    ///@return true (TODO)
    ///
    bool MatchWithGassianNewton(const CloudData::Cloud_Ptr& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                Eigen::Matrix4f& result_pose);

    ///
    ///@brief use PCL built-in ICP api to estimate relative transformatin
    ///
    ///@param input_source input point cloud
    ///@param predict_pose initial pose
    ///@param result_cloud_ptr final estimated source point cloud in target pointcloud frame
    ///@param result_pose estimated pose
    ///
    bool MatchWithPCL(const CloudData::Cloud_Ptr& input_source,
                      const Eigen::Matrix4f& predict_pose,
                      CloudData::Cloud_Ptr& result_cloud_ptr,
                      Eigen::Matrix4f& result_pose);

private:
    MatchMethod match_method_ = MatchMethod::PCL;
    Param param_;
    float score_ = std::numeric_limits<float>::max();

    // target kd tree use for self implemented matching algorithm
    pcl::KdTreeFLANN<CloudData::Point>::Ptr target_kd_tree_ptr_;

    // PCL built-in ICP Intergface
    pcl::IterativeClosestPoint<CloudData::Point, CloudData::Point>::Ptr icp_ptr_;
};

}  // namespace lidar_localization

#endif