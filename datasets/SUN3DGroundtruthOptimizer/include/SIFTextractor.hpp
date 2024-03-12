#ifndef SIFTEXTRACTOR_HPP
#define SIFTEXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

bool ExtractSIFTFeatures(const cv::Mat &image, int minFeatureCount, std::vector<cv::KeyPoint> &keyPoints);

#endif // SIFTEXTRACTOR_HPP