/*
 * @Description:
 * @Created Date: 2020-03-01 18:35:19
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-24 00:36:49
 * @Modified By: Xiaotao Guo
 */

#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}  // namespace lidar_localization