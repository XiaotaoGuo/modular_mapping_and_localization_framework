/*
 * @Description: 3D pose 中位置的先验边
 * @Created Date: 2020-03-01 18:05:35
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-12-19 20:39:31
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_
#define MAPPING_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3PriorXYZ() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}

    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

        // 直接相减求误差
        Eigen::Vector3d estimate = v1->estimate().translation();
        _error = estimate - _measurement;
    }

    // TODO：Buggy
    void linearizeOplus() override {
        // only update position
        const VertexSE3* v = static_cast<const VertexSE3*>(_vertices[0]);
        _jacobianOplusXi.block<3, 3>(0, 0) = v->estimate().rotation();
        _jacobianOplusXi.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    }

    void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

    virtual bool read(std::istream& is) override {
        // 读入观测
        Eigen::Vector3d v;
        is >> v(0) >> v(1) >> v(2);

        setMeasurement(Eigen::Vector3d(v));

        // 读入信息矩阵(只读上三角部分)
        for (int i = 0; i < information().rows(); ++i) {
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j) {
                    information()(j, i) = information()(i, j);
                }
            }
        }
        return true;
    }

    virtual bool write(std::ostream& os) const override {
        // 写入观测
        Eigen::Vector3d v = _measurement;
        os << v(0) << " " << v(1) << " " << v(2) << " ";

        // 写入信息矩阵
        for (int i = 0; i < information().rows(); ++i) {
            for (int j = i; j < information().cols(); ++j) {
                os << " " << information()(i, j);
            }
        }

        return os.good();
    }
};
}  // namespace g2o

#endif
