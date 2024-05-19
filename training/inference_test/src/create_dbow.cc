// #include <algorithm>  // For std::shuffle
// #include <random>     // For std::default_random_engine
// #include <chrono>     // For system clock (seed generation)
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
// OpenCV
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
// #include <DBoW2.h>
// #include <DBoW2/FModel256.h>
#include <TemplatedVocabulary.h>
#include <FORB.h>
#include "Inference.hpp"

using namespace DBoW2;
// using namespace DUtils;
// typedef DBoW2::TemplatedVocabulary<DBoW2::FModel256::TDescriptor, DBoW2::FModel256>
//     ModelVocabulary;

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);
// void createVocabularyFile(ModelVocabulary &voc, std::string &fileName, const std::vector<std::vector<cv::Mat>> &features);
void createVocabularyFile(ORBVocabulary &voc, std::string &fileName, const std::vector<std::vector<cv::Mat>> &features);
// void matToVector(const cv::Mat &mat, std::vector<float> &vec)
// {
//     vec.assign((float *)mat.datastart, (float *)mat.dataend);
// }
// ----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "wrong set of arguments, expected:" << std::endl
                  << argv[0] << " <model_path> <txt_path>" << std::endl;
        return EXIT_FAILURE;
    }
    const std::string keypoint_predictor_path = argv[1];
    const std::string imagePathFile = argv[2];

    // load trained model
    const Extractor sp(keypoint_predictor_path);

    std::vector<std::string> imagePaths;
    std::string line;
    std::ifstream file(imagePathFile);

    if (file.is_open())
    {
        while (std::getline(file, line))
        {
            imagePaths.push_back(line);
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file: " << imagePathFile << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<std::vector<cv::Mat>> features;
    features.clear();
    features.reserve(imagePaths.size());
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    // Process each image
    for (const auto &path : imagePaths)
    {
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty())
        {
            std::cerr << "Failed to load image at: " << path << std::endl;
            continue;
        }
        cv::Mat rgb;
        cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

        
        std::vector<cv::KeyPoint> keypoints;
        orb->detect(rgb, keypoints);

        // cv::Mat mask;

        // cv::Mat descriptors_ORB;

        // orb->detectAndCompute(rgb, mask, keypoints, descriptors_ORB);

        Eigen::MatrixX2d coord;
        Matrix8u descriptors;
        std::vector<cv::KeyPoint> valid_keypoints;
        std::tie(coord, descriptors,valid_keypoints) = sp.getFeatures(rgb,keypoints);
        descriptors.cast<uint8_t>();
        cv::Mat desc;
        cv::eigen2cv(Matrix8u(descriptors.rightCols(descriptors.cols())), desc);
        desc.convertTo(desc, CV_8UC1);
        cv::Mat packedDesc(desc.rows, 32, CV_8UC1); // Create a new matrix with 32 columns

        for (int i = 0; i < desc.rows; i++) {
            for (int j = 0; j < 32; j++) {
                uint8_t byte = 0;
                for (int bit = 0; bit < 8; bit++) {
                    byte |= (desc.at<uint8_t>(i, j * 8 + bit) << bit);
                }
                packedDesc.at<uint8_t>(i, j) = byte;
            }
        }
        // Convert descriptors to correct type and store
        // std::vector<cv::Mat> descriptorList;

        // changeStructure(desc, descriptorList);
        // features.push_back(descriptorList);

        
        features.push_back(std::vector<cv::Mat >());
        changeStructure(packedDesc, features.back());

        std::cout << "Processed features for image: " << path << std::endl;
        // std::cout << "Descriptor List Size: " << features.back().size() << std::endl;

        // if (!features.back().empty())
        // {
        //     std::cout << "Size of first descriptor matrix: " << features.back()[0].rows << "x" << features.back()[0].cols << std::endl;
        // }
    }

    std::cout << "... Extraction done!" << std::endl;

    // define vocabulary
    const int nLevels = 6;
    const int k = 10; // branching factor
    const WeightingType weight = TF_IDF;
    const ScoringType score = L1_NORM;
    // ModelVocabulary voc(k, nLevels, weight, score);
    ORBVocabulary voc(k, nLevels, weight, score);

    std::string vocName = "binary_model_voc.txt";
    createVocabularyFile(voc, vocName, features);

    std::cout << "--- THE END ---" << std::endl;

    return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for (int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i).clone();
    }
}

// ----------------------------------------------------------------------------

//void createVocabularyFile(ModelVocabulary &voc, std::string &fileName, const std::vector<std::vector<cv::Mat>> &cvFeatures) {
    // std::vector<std::vector<std::vector<float>>> features;
    // features.resize(cvFeatures.size());

    // // Seed with a real random value, if available
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine engine(seed);

    // for (size_t i = 0; i < cvFeatures.size(); ++i) {
    //     features[i].resize(cvFeatures[i].size());
    //     for (size_t j = 0; j < cvFeatures[i].size(); ++j) {
    //         matToVector(cvFeatures[i][j], features[i][j]);
    //         if (features[i][j].size() > 200) {
    //             std::shuffle(features[i][j].begin(), features[i][j].end(), engine);
    //             features[i][j].resize(200);  // Keep only the first 500 shuffled elements
    //         }
    //     }
    // }
void createVocabularyFile(ORBVocabulary &voc, std::string &fileName, const std::vector<std::vector<cv::Mat>> &cvFeatures) {
    for (const auto& feature_vector : cvFeatures) {
    if (feature_vector.empty()) {
        std::cerr << "Empty feature vector detected!" << std::endl;
        return;
    }
    for (const auto& desc : feature_vector) {
        if (desc.empty() || desc.type() != CV_8U) {
            std::cerr << "Invalid descriptor detected!" << std::endl;
            return;
        }
    }
}
    std::cout << "> Creating vocabulary. May take some time ..." << std::endl;
    voc.create(cvFeatures);
    std::cout << "... done!" << std::endl;

    std::cout << "> Vocabulary information: " << std::endl
              << voc << std::endl
              << std::endl;

    // save the vocabulary to disk
    std::cout << std::endl
              << "> Saving vocabulary..." << std::endl;
    // voc.save(fileName);
    voc.saveToTextFile(fileName);
    std::cout << "... saved to file: " << fileName << std::endl;
}