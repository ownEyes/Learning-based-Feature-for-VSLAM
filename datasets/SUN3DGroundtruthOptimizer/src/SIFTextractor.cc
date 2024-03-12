#include <SIFTextractor.hpp>

using namespace std;

bool ExtractSIFTFeatures(const cv::Mat &image, int minFeatureCount, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::Feature2D> sift = cv::SIFT::create();

    sift->detect(image, keyPoints);

    return keyPoints.size() > minFeatureCount;
}
