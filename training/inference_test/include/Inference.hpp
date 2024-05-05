#ifndef INFERENCE_HPP
#define INFERENCE_HPP

#include <memory>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <filesystem>
#include <torch/torch.h>
#include <torch/script.h>  // Include the TorchScript headers
#include <torch/csrc/api/include/torch/cuda.h>

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8u;

struct ExtractorImpl {
    torch::Device device;
    torch::jit::script::Module module;

    ExtractorImpl(const std::string& model_path)
        : device(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU) {
        if (!std::filesystem::is_regular_file(model_path)) {
            throw std::filesystem::filesystem_error("Invalid model file", model_path, std::make_error_code(std::errc::no_such_file_or_directory));
        }
        module = torch::jit::load(model_path, device);

#ifdef BENCHMARK
        at::globalContext().setBenchmarkCuDNN(true);
#endif
    }
};

class Extractor {
public:
    // Use explicit to prevent implicit conversions
    explicit Extractor(const std::string &model_path);

    // Utilize the default destructor, copy and move operations provided by the compiler
    ~Extractor() = default;
    Extractor(const Extractor&) = delete;
    Extractor& operator=(const Extractor&) = delete;
    Extractor(Extractor&&) noexcept = default;
    Extractor& operator=(Extractor&&) noexcept = default;

    // Const correctness and pass by const reference
    //std::tuple<Eigen::MatrixX2d, Eigen::MatrixXd> getFeatures(const cv::Mat &image) const;
    std::tuple<Eigen::MatrixX2d, Matrix8u, std::vector<cv::KeyPoint>> getFeatures(const cv::Mat &image,const std::vector<cv::KeyPoint>& keypoints) const;
    // Setter for the minimum score threshold
    void setScoreMin(float min_score);

private:
    std::unique_ptr<ExtractorImpl> impl; // Smart pointer for automatic memory management
    float point_min_score;
};

// void extractFeatures(const Extractor& sp, const std::vector<std::string>& image_list, 
//                      std::vector<std::vector<cv::KeyPoint>>& keypointsList, 
//                      std::vector<cv::Mat>& descriptorsList);

#endif //INFERENCE_HPP