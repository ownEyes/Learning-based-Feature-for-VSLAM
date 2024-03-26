#include <ICP_BAsolver.hpp>

void bundleAdjustmentICP(
    const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> &matchedPoints,
    const extrinsic &initialGuess,
    extrinsic &optimizedPose)
{

    // Initialize g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer.setAlgorithm(solver);

    // Add the single pose vertex
    VertexRePose *poseVertex = new VertexRePose();
    poseVertex->setId(0);
    poseVertex->setEstimate(Sophus::SE3d(initialGuess.R_mat, initialGuess.t));
    optimizer.addVertex(poseVertex);

    // Add 3D point vertices and edges for matched points
    for (size_t j = 0; j < matchedPoints.first.size(); ++j)
    {
        const Eigen::Vector3d &pointCam1 = matchedPoints.first[j];
        const Eigen::Vector3d &pointCam2 = matchedPoints.second[j];

        Vertex3DPoint *pointVertex = new Vertex3DPoint();
        pointVertex->setId(j + 1); // ID is offset by 1 because 0 is used by pose vertex
        pointVertex->setEstimate(pointCam1);
        // pointVertex->setMarginalized(true);
        optimizer.addVertex(pointVertex);

        Edge3DPointPoseAdjustment *edge = new Edge3DPointPoseAdjustment(pointCam1);
        edge->setId(j);
        edge->setVertex(0, dynamic_cast<Vertex3DPoint *>(pointVertex));
        edge->setVertex(1, dynamic_cast<VertexRePose *>(poseVertex)); // Connect to the pose vertex
        edge->setMeasurement(pointCam2);
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
    }

    int maxIterations = -1;           // Use -1 to indicate the setup should be based on gain
    int maxIterationsWithGain = 1000; // Maximum iterations if using gain-based termination
    double gain = 1e-15;              // Termination gain threshold

    if (maxIterations < 0)
    {
        std::cerr << "# Setup termination criterion based on the gain of the iteration" << std::endl;
        g2o::SparseOptimizerTerminateAction *terminateAction = new g2o::SparseOptimizerTerminateAction;
        terminateAction->setGainThreshold(gain);
        terminateAction->setMaxIterations(maxIterationsWithGain);
        optimizer.addPostIterationAction(terminateAction);
    }

    // Set the first pose as fixed to anchor the map
    // optimizer.vertex(0)->setFixed(true);

    // Perform optimization
    optimizer.initializeOptimization();
    int ret = optimizer.optimize(maxIterations);

    if (ret >= 0)
    {
        std::cout << "ICP Optimization success. Iterations: " << ret << std::endl;
    }
    else
    {
        std::cerr << "ICP Optimization failed." << std::endl;
        return;
    }

    // Retrieve the optimized pose
    Sophus::SE3d optimizedPoseSE3 = poseVertex->estimate();
    optimizedPose.set_from_se3(Sophus::SE3d(optimizedPoseSE3.rotationMatrix(), optimizedPoseSE3.translation()));
}

void bundleAdjustmentICP_PoseOnly(
    const std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> &matchedPoints,
    const extrinsic &initialGuess,
    extrinsic &optimizedPose)
{

    // Initialize g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer.setAlgorithm(solver);

    // Add the single pose vertex
    VertexRePose *poseVertex = new VertexRePose();
    poseVertex->setId(0);
    poseVertex->setEstimate(Sophus::SE3d(initialGuess.R_mat, initialGuess.t));
    optimizer.addVertex(poseVertex);

    // Add 3D point vertices and edges for matched points
    for (size_t j = 0; j < matchedPoints.first.size(); ++j)
    {
        const Eigen::Vector3d &pointCam1 = matchedPoints.first[j];
        const Eigen::Vector3d &pointCam2 = matchedPoints.second[j];

        EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(pointCam1);
        edge->setId(j);
        //Connect to the pose vertex
        edge->setVertex(0, dynamic_cast<VertexRePose *>(poseVertex)); 
        edge->setMeasurement(pointCam2);
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
    }

    int maxIterations = -1;           // Use -1 to indicate the setup should be based on gain
    int maxIterationsWithGain = 1000; // Maximum iterations if using gain-based termination
    double gain = 1e-15;              // Termination gain threshold

    if (maxIterations < 0)
    {
        std::cerr << "# Setup termination criterion based on the gain of the iteration" << std::endl;
        g2o::SparseOptimizerTerminateAction *terminateAction = new g2o::SparseOptimizerTerminateAction;
        terminateAction->setGainThreshold(gain);
        terminateAction->setMaxIterations(maxIterationsWithGain);
        optimizer.addPostIterationAction(terminateAction);
    }

    // Set the first pose as fixed to anchor the map
    // optimizer.vertex(0)->setFixed(true);

    // Perform optimization
    optimizer.initializeOptimization();
    int ret = optimizer.optimize(maxIterations);

    if (ret >= 0)
    {
        std::cout << "# ICP Optimization success. Iterations: " << ret << std::endl;
    }
    else
    {
        std::cerr << "# ICP Optimization failed." << std::endl;
        return;
    }

    // Retrieve the optimized pose
    Sophus::SE3d optimizedPoseSE3 = poseVertex->estimate();
    optimizedPose.set_from_se3(Sophus::SE3d(optimizedPoseSE3.rotationMatrix(), optimizedPoseSE3.translation()));
}