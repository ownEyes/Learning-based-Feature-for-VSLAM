#include <BundleAdjustment.hpp>

using namespace g2o;

void bundleAdjustment(const std::vector<Eigen::Vector3d> &worldPoints,
                      const std::vector<std::vector<cv::KeyPoint>> &observations,
                      const std::vector<extrinsic> &poses,
                      const cv::Mat &K,
                      std::vector<int> &pointIndices,
                      std::vector<int> &camIndices,
                      std::vector<extrinsic> &new_poses)
{
    // Initialize the sparse optimizer
    SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // Choose the linear solver and block solver
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Correctly create the linear solver using std::make_unique
    auto linearSolver = std::make_unique<LinearSolverType>();

    // Create the block solver using std::make_unique and move the linear solver into it
    auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));

    auto algorithm = new OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer.setAlgorithm(algorithm);

    // Add camera pose vertices (extrinsics)
    for (size_t i = 0; i < poses.size(); ++i)
    {
        VertexPose *v_se3 = new VertexPose();
        v_se3->setId(i);

        #ifdef DEBUG
        Eigen::Matrix3d rotation_matrix = poses[i].get_se3().rotationMatrix().cast<double>();
        std::cout << "Rotation Matrix: \n"
                  << rotation_matrix << std::endl;

        Eigen::Vector3d translation_vector = poses[i].get_se3().translation().cast<double>();
        std::cout << "Translation Vector: \n"
                  << translation_vector << std::endl;

        v_se3->setEstimate(Sophus::SE3d(rotation_matrix, translation_vector));
        #else
        v_se3->setEstimate(Sophus::SE3d(poses[i].R_mat, poses[i].t));
        #endif
        
        optimizer.addVertex(v_se3);
    }

    // Add 3D point vertices
    int pointIdOffset = poses.size();
    for (size_t i = 0; i < worldPoints.size(); ++i)
    {
        VertexPoint *v_p = new VertexPoint();
        v_p->setId(pointIdOffset + i);
        v_p->setEstimate(worldPoints[i]);
        v_p->setMarginalized(true);
        optimizer.addVertex(v_p);
    }

    for (size_t i = 0; i < observations.size(); ++i)
    {
        int camId = camIndices[i];
        int pointId = pointIndices[i] + pointIdOffset;
        const cv::KeyPoint &kp = observations[camId][i];

        EdgeProjectXYZ2PosePoint *edge = new EdgeProjectXYZ2PosePoint(worldPoints[pointIndices[i]], K);
        edge->setVertex(0, dynamic_cast<VertexPose *>(optimizer.vertex(camId)));
        edge->setVertex(1, dynamic_cast<VertexPoint *>(optimizer.vertex(pointId)));
        edge->setMeasurement(Eigen::Vector2d(kp.pt.x, kp.pt.y));
        // Assuming the information matrix is the identity matrix
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber);
        optimizer.addEdge(edge);
    }

    int maxIterations = -1;           // Use -1 to indicate the setup should be based on gain
    int maxIterationsWithGain = 1000; // Maximum iterations if using gain-based termination
    double gain = 1e-15;              // Termination gain threshold

    if (maxIterations < 0)
    {
        std::cerr << "# Setup termination criterion based on the gain of the iteration" << std::endl;
        g2o::SparseOptimizerTerminateAction *terminateAction = new SparseOptimizerTerminateAction;
        terminateAction->setGainThreshold(gain);
        terminateAction->setMaxIterations(maxIterationsWithGain);
        optimizer.addPostIterationAction(terminateAction);
    }

    optimizer.initializeOptimization();
    int ret= optimizer.optimize(maxIterations); // maxIterations can be set to a default value or based on other criteria

    if(ret >= 0){
        std::cout<< "TIMES OF Global BA OPTIMIZATION: "<< ret <<std::endl;
    }
    else{
        std::cerr << "Global BA OPTIMIZATION FAILED." << std::endl;
        return;
    }
    

    new_poses.clear();              // Clearing the vector to store new poses
    new_poses.resize(poses.size()); // Resize to match the number of camera poses

    for (size_t i = 0; i < poses.size(); ++i)
    {
        VertexPose *poseVertex = dynamic_cast<VertexPose *>(optimizer.vertex(i));
        if (poseVertex)
        {
            Sophus::SE3d optimizedPose = poseVertex->estimate();

            // Convert the optimized pose to your 'extrinsic' format
            extrinsic newExtrinsic;
            newExtrinsic.set_from_se3(Sophus::SE3d(optimizedPose.rotationMatrix(), optimizedPose.translation()));

            // Store the converted pose
            new_poses[i] = newExtrinsic;
        }
    }
}