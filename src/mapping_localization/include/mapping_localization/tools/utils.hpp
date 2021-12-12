/*
 * @Description:
 * @Created Date: 2021-12-10 13:59:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-11 18:18:43
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TOOLS_UTILS_HPP_
#define MAPPING_LOCALIZATION_TOOLS_UTILS_HPP_

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace mapping_localization {

template <typename T>
bool try_load_param(const YAML::Node& node, std::string name, T& value, T default_value) {
    try {
        value = node[name].as<T>();
    } catch (...) {
        std::cout << "Param " << name << "couldn't be found in yaml file, check your config!. Using default value"
                  << default_value << "\n";
        value = default_value;
        return false;
    }

    return true;
}

Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);

}  // namespace mapping_localization

#endif
