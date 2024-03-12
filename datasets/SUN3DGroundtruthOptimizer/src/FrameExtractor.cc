#include <FrameExtractor.hpp>

using namespace std;

void ExtractFrames(string local_dir)
{
    string local_camera = local_dir + "intrinsics.txt";
    string local_image = local_dir + "image/";
    string local_depth = local_dir + "depth/";
    string local_extrinsic = local_dir + "extrinsics/";

    int i_ret;
    float ff;
    FILE *fp = fopen(local_camera.c_str(), "r");
    i_ret = fscanf(fp, "%f", &cam_K.fx);
    i_ret = fscanf(fp, "%f", &ff);
    i_ret = fscanf(fp, "%f", &cam_K.cx);
    i_ret = fscanf(fp, "%f", &ff);
    i_ret = fscanf(fp, "%f", &cam_K.fy);
    i_ret = fscanf(fp, "%f", &cam_K.cy);
    fclose(fp);

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

    std::ofstream imageFile("extracted_images.txt");
    std::ofstream depthFile("extracted_depths.txt");
    std::ofstream extrinsicFile("extracted_extrinsics.txt");

    if (!imageFile.is_open() || !depthFile.is_open() || !extrinsicFile.is_open())
    {
        std::cerr << "Failed to open one or more output files!" << std::endl;
        //return 1;
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
        extrinsicFile << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " "
                      << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " "
                      << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " "
                      << pose.t[0] << " " << pose.t[1] << " " << pose.t[2] << "\n";
    }

    imageFile.close();
    depthFile.close();
    extrinsicFile.close();
}