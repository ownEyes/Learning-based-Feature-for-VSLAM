#ifndef BUNDLEADJUSTMENT_HPP
#define BUNDLEADJUSTMENT_HPP

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <Dataloader.hpp>

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Sophus::SE3d();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &is) override
    {
        // Implement reading function if necessary
        return true;
    }

    virtual bool write(std::ostream &os) const override
    {
        // Implement writing function if necessary
        return true;
    }
};

class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Eigen::Vector3d::Zero();
    }

    virtual void oplusImpl(const double *update) override
    {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &is) override
    {
        // Implement reading function if necessary
        return true;
    }

    virtual bool write(std::ostream &os) const override
    {
        // Implement writing function if necessary
        return true;
    }
};

class EdgeProjectXYZ2PosePoint : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZ2PosePoint(const Eigen::Vector3d &point, const cv::Mat &K) : _point(point)
    {
        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
    }

    virtual void computeError() override
    {
        const VertexPose *vPose = static_cast<const VertexPose *>(_vertices[0]);
        const VertexPoint *vPoint = static_cast<const VertexPoint *>(_vertices[1]);

        Sophus::SE3d pose = vPose->estimate();
        Eigen::Vector3d point = vPoint->estimate();
        Eigen::Vector3d projected = pose.inverse() * point; // Project point into camera coordinates

        // Project to 2D using the camera intrinsic parameters
        Eigen::Vector2d predicted(
            fx * projected[0] / projected[2] + cx,
            fy * projected[1] / projected[2] + cy);

        _error = _measurement - predicted; // Difference between predicted and observed
    }

    virtual void linearizeOplus() override
    {
        VertexPose *vPose = static_cast<VertexPose *>(_vertices[0]);
        VertexPoint *vPoint = static_cast<VertexPoint *>(_vertices[1]);

        Sophus::SE3d pose = vPose->estimate();
        Eigen::Vector3d point = vPoint->estimate();
        Eigen::Vector3d pos_cam = pose.inverse() * point;

        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18); // Adding a small value to avoid division by zero
        double Zinv2 = Zinv * Zinv;

        // Jacobian with respect to the camera pose
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;

        // Jacobian with respect to the 3D point
        Eigen::Matrix<double, 2, 3> jacobian_point = _jacobianOplusXi.block<2, 3>(0, 0);

        _jacobianOplusXj = jacobian_point * pose.inverse().rotationMatrix();
    }

    virtual bool read(std::istream &is) override
    {
        // Implement reading function if necessary
        return true;
    }

    virtual bool write(std::ostream &os) const override
    {
        // Implement writing function if necessary
        return true;
    }

protected:
    Eigen::Vector3d _point; // The 3D point in the world coordinate
    double fx, fy, cx, cy;  // Intrinsic camera parameters
};

void bundleAdjustment(const std::vector<Eigen::Vector3d> &worldPoints,
                      const std::vector<std::vector<cv::KeyPoint>> &observations,
                      const std::vector<extrinsic> &poses,
                      const cv::Mat &K,
                      std::vector<int> &pointIndices,
                      std::vector<int> &camIndices,
                      std::vector<extrinsic> &new_poses);

#endif //BUNDLEADJUSTMENT_HPP