/*
 * @Description:
 * @Created Date: 2021-12-10 13:59:42
 * @Author: Xiaotao Guo
 * -----
 * @Last Modified: 2021-12-13 17:51:37
 * @Modified By: Xiaotao Guo
 */

#include "mapping_localization/tools/utils.hpp"

namespace mapping_localization {

Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= noise(i);
    }
    return information_matrix;
}

Eigen::MatrixXd CalculateSqrtDiagMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= std::sqrt(noise(i));
    }
    return information_matrix;
}

Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
}
}  // namespace mapping_localization
