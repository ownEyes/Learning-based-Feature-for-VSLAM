#ifndef FRAMEEXTRACTOR_HPP
#define FRAMEEXTRACTOR_HPP

#include <mutex>
#include <iomanip>
#include <ThreadPool.hpp>
#include <Eigen/Dense>

#include <opencv2/calib3d.hpp>

#include <Dataloader.hpp>
#include <SIFTextractor.hpp>
#include <BundleAdjustment.hpp>
#include <SuperPointInference.hpp>
#include <ICP_BAsolver.hpp>

void ExtractFrames(std::string local_dir, std::string model_path);

std::vector<extrinsic> calculateRelativeTransformations(const std::vector<extrinsic> &optimized_poses);

std::vector<extrinsic> calculateGlobalPoses(const std::vector<extrinsic> &relative_transformations,
                                            const Sophus::SE3d &initial_pose = Sophus::SE3d());

std::vector<std::vector<cv::DMatch>> matchAndRefine(const std::vector<std::vector<cv::KeyPoint>> &keypointsList,
                                                    const std::vector<cv::Mat> &descriptorsList);

void KeypointTo3D(std::vector<std::vector<cv::KeyPoint>> &kpts,
                  std::vector<std::string> &depth_list,
                  std::vector<extrinsic> &poses,
                  cv::Mat K,
                  std::vector<Eigen::Vector3d> &worldpoints,
                  std::vector<int> &pointIndices,
                  std::vector<int> &camIndices);

void MatchedKeypointsTo3D(const std::vector<std::vector<cv::KeyPoint>> &keypointsList,
                          const std::vector<std::vector<cv::DMatch>> &matchesList,
                          const std::vector<std::string> &depthList,
                          const cv::Mat &K,
                          std::vector<std::pair<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>, bool>> &matchedPointsList);

#endif // FRAMEEXTRACTOR_HPP