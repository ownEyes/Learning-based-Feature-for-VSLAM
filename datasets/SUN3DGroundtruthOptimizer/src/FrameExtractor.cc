#include <FrameExtractor.hpp>

using namespace std;

void ExtractFrames(string local_dir)
{
    string local_camera = local_dir + "intrinsics.txt";
    string local_image = local_dir + "image/";
    string local_depth = local_dir + "depth/";
    string local_extrinsic = local_dir + "extrinsics/";

    cv::Mat K;
    GetCameraK(local_camera, K);

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

    for (size_t i = 0; i < indices.size(); ++i)
    {
        sorted_extracted_image_list[i] = extracted_image_list[indices[i]];
        sorted_extracted_depth_list[i] = extracted_depth_list[indices[i]];
        sorted_extracted_extrinsic_poses[i] = extracted_extrinsic_poses[indices[i]];
    }

    extracted_image_list = std::move(sorted_extracted_image_list);
    extracted_depth_list = std::move(sorted_extracted_depth_list);
    extracted_extrinsic_poses = std::move(sorted_extracted_extrinsic_poses);

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
        extrinsicFile << pose.R[0][0] << " " << pose.R[0][1] << " " << pose.R[0][2] << " " << pose.translation[0] << " "
                      << pose.R[1][0] << " " << pose.R[1][1] << " " << pose.R[1][2] << " " << pose.translation[1] << " "
                      << pose.R[2][0] << " " << pose.R[2][1] << " " << pose.R[2][2] << " " << pose.translation[2] << "\n";
    }

    imageFile.close();
    depthFile.close();
    extrinsicFile.close();
}