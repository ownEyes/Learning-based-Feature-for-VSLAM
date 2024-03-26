#include <SuperPointInference.hpp>

// #define BENCHMARK

#ifdef BENCHMARK
#include <iostream>
#include <chrono>

struct Benchmark
{
    std::string operation;
    std::chrono::steady_clock::time_point start;

    Benchmark(const std::string &op) : operation(op), start(std::chrono::steady_clock::now()) {}

    ~Benchmark()
    {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << operation << " (ms): " << elapsed.count() << std::endl;
    }
};
#endif

SuperPoint::SuperPoint(const std::string &model_path)
    : impl(std::make_unique<SuperPointImpl>(model_path)), // Use std::make_unique for memory safety
      point_min_score(0.015)
{
}

std::tuple<Eigen::MatrixX2d, Eigen::MatrixXd> SuperPoint::getFeatures(const cv::Mat &image) const
{
#ifdef BENCHMARK
    Benchmark total_benchmark("TOTAL");
#endif

    // convert RGB to grey-scale image in [0,1]
    cv::Mat img;
#ifdef BENCHMARK
    Benchmark input_load_benchmark("input load");
#else
    cv::cvtColor(image, img, cv::COLOR_RGB2GRAY);
    img.convertTo(img, CV_32F);
    img /= 255;
#endif

#ifdef BENCHMARK
    input_load_benchmark.~Benchmark(); // Explicitly call the destructor to mark the end of this benchmark section
#endif

    // Check that image dimensions are divisible by cell size
    static constexpr int cell = 8;
    if (img.rows % cell != 0 || img.cols % cell != 0)
    {
        throw std::runtime_error("Image dimensions (" + std::to_string(img.cols) + " x " + std::to_string(img.rows) + ") must be multiple of cell size (" + std::to_string(cell) + ")");
    }

#ifdef BENCHMARK
    Benchmark inference_benchmark("inference");
#endif
    const torch::Tensor input = torch::from_blob(img.data, {1, 1, img.rows, img.cols}, torch::kF32).to(impl->device);
    const auto out = impl->module.forward({input}).toTuple();

#ifdef BENCHMARK
    inference_benchmark.~Benchmark(); // Explicitly call the destructor to mark the end of this benchmark section
#endif
    torch::Tensor tensor_semi = out->elements()[0].toTensor(); // heatmap
    torch::Tensor tensor_feat = out->elements()[1].toTensor(); // descriptors

#ifdef BENCHMARK
    Benchmark heatmap_benchmark("heatmap");
#endif

    // softmax
    tensor_semi = torch::nn::functional::softmax(tensor_semi.squeeze(0), 0).permute({1, 2, 0});

    // reshape to original input image dimension
    const torch::Tensor heatmap = tensor_semi
                                      .slice(2, 0, -1) // remove dust bin
                                      .reshape({img.rows / cell, img.cols / cell, cell, cell})
                                      .permute({0, 2, 1, 3})
                                      .reshape({img.rows, img.cols});

    // suppress non-maxima in local 3x3 neighbourhood
    // TODO: make 'kernel_size' of 3 configurable
    const torch::Tensor heatmap_nms = torch::nn::functional::max_pool2d(
                                          heatmap.unsqueeze(0).unsqueeze(1),
                                          torch::nn::MaxPool2dOptions({3, 3}).stride({1, 1}).padding(1))
                                          .squeeze(0)
                                          .squeeze(0);

    const torch::Tensor mask_nms = torch::logical_and((heatmap > point_min_score), (heatmap == heatmap_nms));
#ifdef BENCHMARK
    heatmap_benchmark.~Benchmark();
#endif

#ifdef BENCHMARK
    Benchmark keypoint_benchmark("keypoint");
#endif

    // point coordinates in normalised [0,1] image space (x,y)
    const torch::Tensor pts_norm = torch::roll(torch::nonzero(mask_nms).to(torch::kDouble) / torch::tensor(input.sizes().slice(2, 2), impl->device), 1, 1);

    // centred point coordinates normalised in [-1,+1]
    const torch::Tensor pts_norm_c = (pts_norm * 2 - 1).to(torch::kFloat);

    // get descriptors at keypoint coordinates
    torch::Tensor desc = torch::nn::functional::grid_sample(
                             tensor_feat, pts_norm_c.unsqueeze(0).unsqueeze(0),
                             torch::nn::functional::GridSampleFuncOptions().align_corners(false))
                             .squeeze(0)
                             .squeeze(1)
                             .transpose(1, 0)
                             .to(torch::kDouble)
                             .cpu();

    assert(pts_norm.sizes()[0] == desc.sizes()[0]);

    // normalise
    desc = desc / torch::norm(desc, 2, 1, true);
#ifdef BENCHMARK
    keypoint_benchmark.~Benchmark();
#endif

#ifdef BENCHMARK
    Benchmark format_benchmark("format");
#endif

    // N x 2 keypoint coordinates {(x_0, y_0), ..., (x_N, y_N)}
    const Eigen::MatrixX2d coord = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>>(pts_norm.cpu().contiguous().data_ptr<double>(), pts_norm.sizes()[0], pts_norm.sizes()[1]);

    // N x D keypoint descriptors {f_0, ..., f_n} with f as 1 x D row-vector
    const Eigen::MatrixXd descr = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(desc.contiguous().data_ptr<double>(), desc.sizes()[0], desc.sizes()[1]);
#ifdef BENCHMARK
    format_benchmark.~Benchmark();
#endif

#ifdef BENCHMARK
    total_benchmark.~Benchmark();
#endif

    return {coord, descr};
}

void SuperPoint::setScoreMin(float min_score)
{
    this->point_min_score = min_score;
}

void extractSPFeatures(const SuperPoint &sp, const std::vector<std::string> &image_list,
                       std::vector<std::vector<cv::KeyPoint>> &keypointsList,
                       std::vector<cv::Mat> &descriptorsList)
{
    keypointsList.clear();
    descriptorsList.clear();

    for (const auto &image_path : image_list)
    {
        cv::Mat image = GetImage(image_path);
        if (image.empty())
        {
            std::cerr << "Could not read image: " << image_path << std::endl;
            continue;
        }

        // Convert to RGB as required by the SuperPoint model
        cv::Mat rgbImage;
        cv::cvtColor(image, rgbImage, cv::COLOR_BGR2RGB);

        Eigen::MatrixX2d coords;
        Eigen::MatrixXd descs;
        std::tie(coords, descs) = sp.getFeatures(rgbImage);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::eigen2cv(Eigen::MatrixXd(descs.rightCols(descs.cols() - 2)), descriptors);

        for (int i = 0; i < coords.rows(); ++i)
        {
            // Convert normalized coordinates [0, 1] to pixel coordinates
            float x = coords(i, 0) * image.cols; // Scale x-coordinate by image width
            float y = coords(i, 1) * image.rows; // Scale y-coordinate by image height

            // Convert Eigen::MatrixX2d row to cv::KeyPoint
            keypoints.emplace_back(cv::KeyPoint(x, y, 1.f)); // Assuming size=1 for all keypoints
            // std::cout << "sp keypoint at location :" << x << " " << y << std::endl;
        }

        keypointsList.push_back(keypoints);
        descriptorsList.push_back(descriptors.clone()); // Clone to ensure deep copy
    }
}