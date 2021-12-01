/*
 * @Description: 前端里程计的node文件
 * @Created Date: 2020-02-05 02:56:27
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-29 23:04:51
 * @Modified By: Xiaotao Guo
 */

#include <ros/ros.h>

#include "glog/logging.h"
#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/mapping/front_end/front_end_flow.hpp"

using namespace mapping_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}