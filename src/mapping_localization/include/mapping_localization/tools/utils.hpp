/*
 * @Description:
 * @Created Date: 2021-12-10 13:59:59
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-13 17:09:15
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_TOOLS_UTILS_HPP_
#define MAPPING_LOCALIZATION_TOOLS_UTILS_HPP_

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace mapping_localization {

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii) {
        os << " " << *ii;
    }
    os << "]";
    return os;
}

///
///@brief template for loading param from yaml node and
///
///@tparam T
///@param node
///@param name
///@param value output value, should be with overloaded output stream operator
///@param default_value de
///@param options valid options, if provided
///@return true
///@return false
///
template <typename T>
bool try_load_param(const YAML::Node& node,
                    std::string name,
                    T& value,
                    T default_value,
                    const std::vector<T>& options = std::vector<T>()) {
    bool success = false;
    try {
        value = node[name].as<T>();
        if (options.size()) {
            for (const T& option : options) {
                if (option == value) {
                    success = true;
                    break;
                }
            }
        } else {
            success = true;
        }
    } catch (...) {
        success = false;
    }

    if (!success) {
        std::cout << "Param " << name << " couldn't be found in yaml file or invalid, check your config!.\n";

        if (options.size()) {
            std::cout << "Valid options: ";
            for (const T& option : options) {
                std::cout << option << ", ";
            }
            std::cout << std::endl;
        }

        std::cout << " Using default value: " << default_value << "\n";
        value = default_value;
    }

    return success;
}

Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);

Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
Eigen::MatrixXd CalculateSqrtDiagMatrix(Eigen::VectorXd noise);

}  // namespace mapping_localization

#endif
