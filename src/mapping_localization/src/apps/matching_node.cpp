/*
 * @Description: 地图匹配定位的node文件
 * @Created Date: 2020-02-05 02:56:27
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:33:33
 * @Modified By: Xiaotao Guo
 */

#include <ros/ros.h>

#include "glog/logging.h"
#include "mapping_localization/global_defination/global_defination.h"
#include "mapping_localization/matching/matching_flow.hpp"

using namespace mapping_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}