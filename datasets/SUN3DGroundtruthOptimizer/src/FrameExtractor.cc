#include <FrameExtractor.hpp>

using namespace std;

#ifdef DEBUG
void PrintMat(const cv::Mat &mat)
{
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            // Use mat.at<float>(i, j) if your matrix type is CV_32F
            // For double precision (CV_64F), use mat.at<double>(i, j)
            std::cout << mat.at<float>(i, j) << " ";
        }
        std::cout << std::endl;
    }
}
#endif

void ExtractFrames(string local_dir, string model_path)
{
    string local_camera = local_dir + "intrinsics.txt";
    string local_image = local_dir + "image/";
    string local_depth = local_dir + "depth/";
    string local_extrinsic = local_dir + "extrinsics/";

    cv::Mat K;
    GetCameraK(local_camera, K);

#ifdef DEBUG
    PrintMat(K);
#endif

    vector<string> image_list;
    vector<string> depth_list;
    vector<string> extrinsic_list;

    GetLocalFileNames(local_image, &image_list);

    GetLocalFileNames(local_depth, &depth_list);
    GetLocalFileNames(local_extrinsic, &extrinsic_list);
    AssignDepthList(image_list, &depth_list);

    vector<extrinsic> extrinsic_poses;
    GetExtrinsicData(
        extrinsic_list[extrinsic_list.size() - 1],
        image_list.size(),
        &extrinsic_poses);

    int minFeatureCount = 50;
    vector<vector<cv::KeyPoint>> SIFTs;
    vector<string> extracted_image_list;
    vector<string> extracted_depth_list;
    vector<extrinsic> extracted_extrinsic_poses;
    ThreadPool pool(20);
    mutex siftMutex; // Mutex for protecting access to the SIFTs vector and lists
    mutex listMutex;

    for (size_t i = 0; i < image_list.size(); i += kSampleFactor)
    {
        pool.enqueue([&, i]
                     {
            cv::Mat image_data = GetImage(image_list[i]);
            if (image_data.empty()) {
                std::cerr << "Failed to load image: " << image_list[i] << std::endl;
                return;
            }
            std::vector<cv::KeyPoint> kpts;
            if (ExtractSIFTFeatures(image_data, minFeatureCount, kpts)) {
                {
                    std::lock_guard<std::mutex> guard_sift(siftMutex);
                    SIFTs.push_back(kpts);
                }
                {
                    std::lock_guard<std::mutex> guard_list(listMutex);
                    extracted_image_list.push_back(image_list[i]);
                    extracted_depth_list.push_back(depth_list[i]);
                    extracted_extrinsic_poses.push_back(extrinsic_poses[i]);
                }
            } });
    }

    pool.wait();

    std::vector<size_t> indices(extracted_image_list.size());
    std::iota(indices.begin(), indices.end(), 0); // Fill with 0, 1, ..., indices.size()-1

    std::sort(indices.begin(), indices.end(),
              [&](size_t a, size_t b)
              {
                  std::string name_a = extracted_image_list[a].substr(extracted_image_list[a].find_last_of('/') + 1);
                  name_a = name_a.substr(0, name_a.find('-'));
                  std::string name_b = extracted_image_list[b].substr(extracted_image_list[b].find_last_of('/') + 1);
                  name_b = name_b.substr(0, name_b.find('-'));

                  return std::stoi(name_a) < std::stoi(name_b);
              });

    std::vector<std::string> sorted_extracted_image_list(extracted_image_list.size());
    std::vector<string> sorted_extracted_depth_list(extracted_depth_list.size());
    std::vector<extrinsic> sorted_extracted_extrinsic_poses(extracted_extrinsic_poses.size());
    std::vector<vector<cv::KeyPoint>> sorted_SIFTs(SIFTs.size());

    for (size_t i = 0; i < indices.size(); ++i)
    {
        sorted_extracted_image_list[i] = extracted_image_list[indices[i]];
        sorted_extracted_depth_list[i] = extracted_depth_list[indices[i]];
        sorted_extracted_extrinsic_poses[i] = extracted_extrinsic_poses[indices[i]];
        sorted_SIFTs[i] = SIFTs[indices[i]];
    }

    extracted_image_list = std::move(sorted_extracted_image_list);
    extracted_depth_list = std::move(sorted_extracted_depth_list);
    extracted_extrinsic_poses = std::move(sorted_extracted_extrinsic_poses);
    SIFTs = std::move(sorted_SIFTs);

    std::ofstream imageFile("extracted_images.txt");
    std::ofstream depthFile("extracted_depths.txt");
    std::ofstream extrinsicFile("extracted_extrinsics.txt");

    if (!imageFile.is_open() || !depthFile.is_open() || !extrinsicFile.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        // return 1;
    }

    for (const auto &imageName : extracted_image_list)
    {
        imageFile << imageName << "\n";
    }

    for (const auto &depthName : extracted_depth_list)
    {
        depthFile << depthName << "\n";
    }

    for (const auto &pose : extracted_extrinsic_poses)
    {
        extrinsicFile << std::fixed << std::setprecision(17)
                      << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                      << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                      << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    imageFile.close();
    depthFile.close();
    extrinsicFile.close();

    std::vector<Eigen::Vector3d> points3D;
    std::vector<int> ptIdx;
    std::vector<int> camIdx;

    vector<extrinsic> optimized_poses;

    KeypointTo3D(SIFTs, extracted_depth_list, extracted_extrinsic_poses, K, points3D, ptIdx, camIdx);

    bundleAdjustment(points3D, SIFTs, extracted_extrinsic_poses, K, ptIdx, camIdx, optimized_poses);

    std::ofstream extrinsicFile_new("optimized_extrinsics.txt");
    if (!extrinsicFile_new.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        // return 1;
    }

    for (const auto &pose : optimized_poses)
    {
        extrinsicFile_new << std::fixed << std::setprecision(17)
                          << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                          << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                          << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    extrinsicFile_new.close();

    std::vector<extrinsic> relative_transformations;
    relative_transformations = calculateRelativeTransformations(optimized_poses);

    std::ofstream poseFile("relative_transformations.txt");
    if (!poseFile.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        // return 1;
    }

    for (const auto &pose : relative_transformations)
    {
        poseFile << std::fixed << std::setprecision(17)
                 << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                 << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                 << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    poseFile.close();

    // cout << "start SuperPoint processing" << endl;

    const SuperPoint sp(model_path);
    vector<vector<cv::KeyPoint>> sp_kpts;
    vector<cv::Mat> sp_descs;

    extractSPFeatures(sp, extracted_image_list, sp_kpts, sp_descs);

    std::vector<std::vector<cv::DMatch>> matchesList;

    matchesList = matchAndRefine(sp_kpts, sp_descs);

    std::cout << "matchesList size: " << matchesList.size() << std::endl;

    // Modify the matchedPointsList structure to include a flag
    std::vector<std::pair<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>, bool>> matchedPointsList;

    MatchedKeypointsTo3D(sp_kpts, matchesList, extracted_depth_list, K, matchedPointsList);

    std::vector<extrinsic> refined_relatives;

    refined_relatives.clear(); // Clearing the vector to store new poses
    refined_relatives.resize(matchedPointsList.size());

    std::cout << "matchedPointsList size: " << matchedPointsList.size() << std::endl;

    // int yes=0, no=0;

    for (size_t i = 0; i < matchedPointsList.size(); ++i)
    {
        // Check if there are more than 10 matched points and the flag is set
        if (matchedPointsList[i].second && matchedPointsList[i].first.first.size() > 10)
        { // Check the flag
            extrinsic optimizedPose;
            bundleAdjustmentICP_PoseOnly(matchedPointsList[i].first, relative_transformations[i], optimizedPose); // Pass the pair of points
            refined_relatives[i] = optimizedPose;
            // yes++;
        }
        else
        {
            refined_relatives[i] = relative_transformations[i];
            // no++;
        }
    }
    
    cout << "bundleAdjustmentICP Done." << endl;
    // std::cout << "refined poses size: " << refined_relatives.size() << std::endl;
    // std::cout << "refined poses num: " << yes<< " not refined poses num: " << no << std::endl;

    std::ofstream SPposeFile("refined_relatives.txt");
    if (!SPposeFile.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        // return 1;
    }

    for (const auto &pose : refined_relatives)
    {
        SPposeFile << std::fixed << std::setprecision(17)
                   << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                   << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                   << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    SPposeFile.close();

    Sophus::SE3d initial_pose = optimized_poses[0].get_se3();

    std::vector<extrinsic> refined_poses;

    refined_poses = calculateGlobalPoses(refined_relatives, initial_pose);

    std::ofstream GlobalSPFile("refined_poses.txt");
    if (!GlobalSPFile.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        // return 1;
    }

    for (const auto &pose : refined_poses)
    {
        GlobalSPFile << std::fixed << std::setprecision(17)
                     << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                     << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                     << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    GlobalSPFile.close();
}

std::vector<extrinsic> calculateRelativeTransformations(const std::vector<extrinsic> &optimized_poses)
{
    std::vector<extrinsic> relative_transformations;

    for (size_t i = 0; i < optimized_poses.size() - 1; ++i)
    {
        // Get the SE3 representation of the current and next pose
        Sophus::SE3d pose_current = optimized_poses[i].get_se3();
        Sophus::SE3d pose_next = optimized_poses[i + 1].get_se3();

        // Calculate the relative transformation (pose_next * inverse(pose_current))
        Sophus::SE3d relative_transformation = pose_current.inverse() * pose_next;

        extrinsic re;

        re.set_from_se3(relative_transformation);

        // Store the relative transformation
        relative_transformations.push_back(re);
    }

    return relative_transformations;
}

std::vector<extrinsic> calculateGlobalPoses(const std::vector<extrinsic> &relative_transformations,
                                            const Sophus::SE3d &initial_pose)
{
    std::vector<extrinsic> global_poses;

    // Initialize the first global pose
    Sophus::SE3d global_pose = initial_pose;
    extrinsic initial_extrinsic;

    initial_extrinsic.set_from_se3(global_pose);
    global_poses.push_back(initial_extrinsic);

    // Iterate through the relative transformations
    for (const auto &relative_transformation : relative_transformations)
    {
        // Get the SE3 representation of the relative transformation
        Sophus::SE3d relative_transformation_se3 = relative_transformation.get_se3();

        // Calculate the next global pose by applying the relative transformation
        global_pose = global_pose * relative_transformation_se3;

        // Convert back to the extrinsic type and store it
        extrinsic global_extrinsic;
        global_extrinsic.set_from_se3(global_pose);
        global_poses.push_back(global_extrinsic);
    }

    return global_poses;
}

std::vector<std::vector<cv::DMatch>> matchAndRefine(const std::vector<std::vector<cv::KeyPoint>> &keypointsList,
                                                    const std::vector<cv::Mat> &descriptorsList)
{
    std::vector<std::vector<cv::DMatch>> finalMatches;

    // Ensure there is at least two sets of descriptors for matching
    if (descriptorsList.size() < 2)
    {
        return finalMatches;
    }

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    for (size_t i = 0; i < descriptorsList.size() - 1; ++i)
    {
        cv::Mat descriptors1, descriptors2;
        if (descriptorsList[i].type() != CV_32F)
        {
            descriptorsList[i].convertTo(descriptors1, CV_32F); // Convert to float
        }
        else
        {
            descriptors1 = descriptorsList[i];
        }

        if (descriptorsList[i + 1].type() != CV_32F)
        {
            descriptorsList[i + 1].convertTo(descriptors2, CV_32F); // Convert to float
        }
        else
        {
            descriptors2 = descriptorsList[i + 1];
        }
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2); // 2 nearest neighbors

        std::vector<cv::DMatch> goodMatches;
        for (const auto &m : knnMatches)
        {
            if (m.size() < 2)
                continue; // Safety check

            if (m[0].distance < 0.7f * m[1].distance)
            { // Lowe's ratio test
                goodMatches.push_back(m[0]);
            }
        }

        if (goodMatches.size() >= 4)
        { // RANSAC needs at least 4 points
            std::vector<cv::Point2f> srcPoints, dstPoints;
            for (const auto &match : goodMatches)
            {
                srcPoints.push_back(keypointsList[i][match.queryIdx].pt);
                dstPoints.push_back(keypointsList[i + 1][match.trainIdx].pt);
            }

            cv::Mat mask;                                                  // Inliers and outliers mask
            cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 3, mask); // RANSAC to refine matches

            std::vector<cv::DMatch> refinedMatches;
            for (int j = 0; j < mask.rows; ++j)
            {
                if (mask.at<uchar>(j))
                { // If the match is an inlier
                    refinedMatches.push_back(goodMatches[j]);
                }
            }

            finalMatches.push_back(refinedMatches);
        }
        else
        {
            // If not enough matches to perform RANSAC, consider pushing back goodMatches or leave as empty
            finalMatches.push_back(goodMatches); // No refined matches
        }
    }

    return finalMatches;
}

void KeypointTo3D(std::vector<vector<cv::KeyPoint>> &kpts,
                  std::vector<std::string> &depth_list,
                  std::vector<extrinsic> &poses,
                  cv::Mat K,
                  std::vector<Eigen::Vector3d> &worldpoints,
                  std::vector<int> &pointIndices,
                  std::vector<int> &camIndices)
{
    worldpoints.clear();
    pointIndices.clear();
    camIndices.clear();

    for (size_t imgIndex = 0; imgIndex < kpts.size(); ++imgIndex)
    {
        cv::Mat depthImg = GetDepth(depth_list[imgIndex]); // Use GetDepth to read the depth image

        for (const auto &keypoint : kpts[imgIndex])
        {
            float depth = depthImg.at<float>(keypoint.pt.y, keypoint.pt.x); // Access depth value
            // cout << "depth value: " << depth << endl;

            // Check if depth value is valid (non-zero)
            if (depth <= 0)
            {
                continue; // Skip this keypoint if depth is invalid
            }

            // Convert from 2D pixel coordinates to 3D point in camera coordinates
            Eigen::Vector3d point3D((keypoint.pt.x - K.at<float>(0, 2)) * depth / K.at<float>(0, 0),
                                    (keypoint.pt.y - K.at<float>(1, 2)) * depth / K.at<float>(1, 1),
                                    depth);

            // Transform from camera coordinates to world coordinates using extrinsic parameters
            Sophus::SE3d pose = poses[imgIndex].get_se3();
            Eigen::Vector3d pointWorld = pose * point3D;

            worldpoints.push_back(pointWorld);
            pointIndices.push_back(worldpoints.size() - 1); // Index of the newly added world point
            camIndices.push_back(imgIndex);                 // Index of the camera
        }
    }
    return;
}

void MatchedKeypointsTo3D(const std::vector<std::vector<cv::KeyPoint>> &keypointsList,
                          const std::vector<std::vector<cv::DMatch>> &matchesList,
                          const std::vector<std::string> &depthList,
                          const cv::Mat &K,
                          std::vector<std::pair<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>, bool>> &matchedPointsList)
{
    matchedPointsList.clear();

    // Ensure the depth list has enough entries
    if (depthList.size() < keypointsList.size())
    {
        std::cerr << "Depth list does not have enough entries for all images." << std::endl;
        return;
    }

    // Iterate through all matches between each pair of consecutive images
    for (size_t i = 0; i < matchesList.size(); ++i)
    {
        const auto &matches = matchesList[i];
        cv::Mat depthImg1 = GetDepth(depthList[i]);
        cv::Mat depthImg2 = GetDepth(depthList[i + 1]);

        std::vector<Eigen::Vector3d> pointsCam1, pointsCam2;

        for (const auto &match : matches)
        {
            const auto &kp1 = keypointsList[i][match.queryIdx];
            const auto &kp2 = keypointsList[i + 1][match.trainIdx];

            float depth1 = depthImg1.at<float>(kp1.pt.y, kp1.pt.x);

            float depth2 = depthImg2.at<float>(kp2.pt.y, kp2.pt.x);

            // std::cout<< "depth1: "<<depth1<<" at location :"<<kp1.pt.y<<" "<<kp1.pt.x <<std::endl;
            // std::cout<< "depth2: "<<depth2<<" at location :"<<kp2.pt.y<<" "<<kp2.pt.x <<std::endl;

            if (depth1 <= 0 || depth2 <= 0)
            {
                continue; // Skip if depth is invalid in either image
            }

            Eigen::Vector3d point3D_1((kp1.pt.x - K.at<float>(0, 2)) * depth1 / K.at<float>(0, 0),
                                      (kp1.pt.y - K.at<float>(1, 2)) * depth1 / K.at<float>(1, 1),
                                      depth1);

            Eigen::Vector3d point3D_2((kp2.pt.x - K.at<float>(0, 2)) * depth2 / K.at<float>(0, 0),
                                      (kp2.pt.y - K.at<float>(1, 2)) * depth2 / K.at<float>(1, 1),
                                      depth2);

            pointsCam1.push_back(point3D_1);
            pointsCam2.push_back(point3D_2);
        }

        // std::cout<< "pointsCam1 size: "<<pointsCam1.size()<<std::endl;
        // std::cout<< "pointsCam2 size: "<<pointsCam2.size()<<std::endl;
        // Add the matched points for this pair of images to the list
        if (!pointsCam1.empty() && !pointsCam2.empty())
        {
            matchedPointsList.emplace_back(std::make_pair(pointsCam1, pointsCam2), true); // True indicates this set is for optimization
        }
        else
        {
            // If there are no valid points, you can choose to add an empty pair with a false flag, or simply skip adding
            matchedPointsList.emplace_back(std::make_pair(std::vector<Eigen::Vector3d>(), std::vector<Eigen::Vector3d>()), false);
        }
    }
}
