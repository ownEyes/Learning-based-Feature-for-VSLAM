#ifndef SUPERPOINTINFERENCE_HPP
#define SUPERPOINTINFERENCE_HPP

#include <memory>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <filesystem>
#include <torch/torch.h>
#include <torch/script.h>  // Include the TorchScript headers
#include <torch/csrc/api/include/torch/cuda.h>


#include <Dataloader.hpp>

struct SuperPointImpl {
    torch::Device device;
    torch::jit::script::Module module;

    SuperPointImpl(const std::string& model_path)
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

class SuperPoint {
public:
    // Use explicit to prevent implicit conversions
    explicit SuperPoint(const std::string &model_path);

    // Utilize the default destructor, copy and move operations provided by the compiler
    ~SuperPoint() = default;
    SuperPoint(const SuperPoint&) = delete;
    SuperPoint& operator=(const SuperPoint&) = delete;
    SuperPoint(SuperPoint&&) noexcept = default;
    SuperPoint& operator=(SuperPoint&&) noexcept = default;

    // Const correctness and pass by const reference
    std::tuple<Eigen::MatrixX2d, Eigen::MatrixXd> getFeatures(const cv::Mat &image) const;

    // Setter for the minimum score threshold
    void setScoreMin(float min_score);

private:
    std::unique_ptr<SuperPointImpl> impl; // Smart pointer for automatic memory management
    float point_min_score;
};

void extractSPFeatures(const SuperPoint& sp, const std::vector<std::string>& image_list, 
                     std::vector<std::vector<cv::KeyPoint>>& keypointsList, 
                     std::vector<cv::Mat>& descriptorsList);

#endif //SUPERPOINTINFERENCE_HPP