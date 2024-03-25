#ifndef ICP_BASOLVER_HPP
#define ICP_BASOLVER_HPP

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
// #include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <Dataloader.hpp>

class VertexRePose : public g2o::BaseVertex<6, Sophus::SE3d>
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

class Vertex3DPoint : public g2o::BaseVertex<3, Eigen::Vector3d>
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

class Edge3DPointPoseAdjustment : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, Vertex3DPoint, VertexRePose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Edge3DPointPoseAdjustment(const Eigen::Vector3d &correspondingPoint)
    {
        // // The measurement is the corresponding 3D point in the second measurement
        // setMeasurement(correspondingPoint);
        // // Information matrix defines the confidence in this measurement, usually set to identity for equal confidence
        // setInformation(Eigen::Matrix3d::Identity());
    }

    // Error computation
    virtual void computeError() override
    {
        const VertexRePose *poseVertex = static_cast<const VertexRePose *>(_vertices[1]);
        const Sophus::SE3d &pose = poseVertex->estimate();

        const Vertex3DPoint *pointVertex = static_cast<const Vertex3DPoint *>(_vertices[0]);
        const Eigen::Vector3d &point = pointVertex->estimate();

        // Transform the 3D point by the current pose estimate and compute the error against the observed corresponding point
        _error = _measurement - pose.inverse() * point;
    }

    // Jacobian computation (optional but recommended for optimization performance)
    virtual void linearizeOplus() override
    {
        const VertexRePose *poseVertex = static_cast<const VertexRePose *>(_vertices[1]);
        const Sophus::SE3d &pose = poseVertex->estimate();

        const Vertex3DPoint *pointVertex = static_cast<const Vertex3DPoint *>(_vertices[0]);
        const Eigen::Vector3d &point = pointVertex->estimate();

        Eigen::Vector3d transformedPoint = pose.inverse() * point;

        // For the point vertex
        _jacobianOplusXi = pose.inverse().rotationMatrix();

        // For the pose vertex
        _jacobianOplusXj.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.block<3, 3>(0, 3) = Sophus::SO3d::hat(transformedPoint);
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
    Eigen::Vector3d correspondingPoint;
};

class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexRePose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    virtual void computeError() override
    {
        const VertexRePose *pose = static_cast<const VertexRePose *>(_vertices[0]);
        _error = _measurement - pose->estimate().inverse() * _point;
    }

    virtual void linearizeOplus() override
    {
        VertexRePose *pose = static_cast<VertexRePose *>(_vertices[0]);
        Sophus::SE3d T = pose->estimate();
        Eigen::Vector3d xyz_trans = T.inverse() * _point;
        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
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
    Eigen::Vector3d _point;
};

void bundleAdjustmentICP(
    const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> &matchedPoints,
    const extrinsic &initialGuess,
    extrinsic &optimizedPose);

void bundleAdjustmentICP_PoseOnly(
    const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> &matchedPoints,
    const extrinsic &initialGuess,
    extrinsic &optimizedPose);

#endif