#ifndef FRAMEEXTRACTOR_HPP
#define FRAMEEXTRACTOR_HPP

#include <mutex>
#include <iomanip>
#include <ThreadPool.hpp>
#include <Eigen/Dense>


#include <Dataloader.hpp>
#include <SIFTextractor.hpp>
#include <BundleAdjustment.hpp>

void ExtractFrames(std::string local_dir);

void KeypointTo3D(std::vector<std::vector<cv::KeyPoint>> &kpts,
                  std::vector<std::string> &depth_list,
                  std::vector<extrinsic> &poses,
                  cv::Mat K,
                  std::vector<Eigen::Vector3d> &worldpoints,
                  std::vector<int> &pointIndices,
                  std::vector<int> &camIndices);

std::vector<extrinsic> calculateRelativeTransformations(const std::vector<extrinsic>& optimized_poses);

#endif // FRAMEEXTRACTOR_HPP