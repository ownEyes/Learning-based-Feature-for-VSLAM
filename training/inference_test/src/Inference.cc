#include "Inference.hpp"

//#define BENCHMARK

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

Extractor::Extractor(const std::string &model_path)
    : impl(std::make_unique<ExtractorImpl>(model_path)), // Use std::make_unique for memory safety
      point_min_score(0.99)
{
}

std::tuple<Eigen::MatrixX2d, Matrix8u, std::vector<cv::KeyPoint>> Extractor::getFeatures(const cv::Mat &image,const std::vector<cv::KeyPoint>& keypoints) const
{
#ifdef BENCHMARK
    Benchmark total_benchmark("TOTAL");
#endif

    // convert RGB to grey-scale image in [0,1]
    cv::Mat img;
#ifdef BENCHMARK
    Benchmark input_load_benchmark("input load");
#endif
    cv::cvtColor(image, img, cv::COLOR_RGB2GRAY);
    img.convertTo(img, CV_32F);
    // img /= 255;

    if (img.empty())
    {
        throw std::runtime_error("Preprocessed image is empty.");
    }

    
#ifdef BENCHMARK
    std::cout << "Image size after preprocessing: " << img.rows << "x" << img.cols << std::endl;
    input_load_benchmark.~Benchmark(); // Explicitly call the destructor to mark the end of this benchmark section
#endif

    // Check that image dimensions are divisible by cell size
    // static constexpr int cell = 8;
    // if (img.rows % cell != 0 || img.cols % cell != 0)
    // {
    //     throw std::runtime_error("Image dimensions (" + std::to_string(img.cols) + " x " + std::to_string(img.rows) + ") must be multiple of cell size (" + std::to_string(cell) + ")");
    // }

#ifdef BENCHMARK
    Benchmark inference_benchmark("inference");
#endif
    const torch::Tensor input = torch::from_blob(img.data, {1, 1, img.rows, img.cols}, torch::kF32).to(impl->device);


    if (input.numel() == 0)
    {
        throw std::runtime_error("Input tensor is empty after creation.");
    }

    const auto out = impl->module.forward({input}).toTuple();

#ifdef BENCHMARK
    std::cout << "Input tensor size: " << input.sizes() << std::endl;
    inference_benchmark.~Benchmark(); // Explicitly call the destructor to mark the end of this benchmark section
#endif
    // torch::Tensor tensor_semi = out->elements()[1].toTensor(); // heatmap
    torch::Tensor tensor_feat = out->elements()[0].toTensor(); // descriptors
    torch::Tensor binary_tensor = tensor_feat > 0;

    // Step 2: Convert boolean tensor to integers (0 or 1)
    binary_tensor = binary_tensor.toType(torch::kUInt8);
    

#ifdef BENCHMARK
    Benchmark heatmap_benchmark("heatmap");
    std::cout<<"tensor_feat size: "<<tensor_feat.sizes()<<std::endl;
    // std::cout<<"tensor_semi size: "<<tensor_semi.sizes()<<std::endl;
    // std::cout << "Tensor size before max_pool2d: " << tensor_semi.sizes() << std::endl;
#endif

    
    // if (tensor_semi.numel() == 0)
    // {
    //     throw std::runtime_error("Tensor is empty before max_pool2d.");
    // }

    // torch::Tensor heatmap_bool=(tensor_semi.squeeze(0).squeeze(0) > point_min_score);

    // // Convert heatmap to double precision
    // torch::Tensor heatmap = heatmap_bool.to(torch::kDouble);

    //suppress non-maxima in local 3x3 neighbourhood
    // int nms_kernel_size = 3; // Make this configurable as needed
    // const torch::Tensor heatmap_nms = torch::nn::functional::max_pool2d(
    //                                       heatmap.unsqueeze(0).unsqueeze(1),
    //                                       torch::nn::MaxPool2dOptions({nms_kernel_size, nms_kernel_size}).stride(1).padding(nms_kernel_size / 2))
    //                                       .squeeze(0)
    //                                       .squeeze(0);
    // std::cout<<"desc heatmap_nms: "<<heatmap_nms.sizes()<<std::endl;

    // // const torch::Tensor mask_nms = torch::logical_and((tensor_semi.squeeze(0).squeeze(0) > point_min_score), (tensor_semi.squeeze(0).squeeze(0) == heatmap_nms));
    // torch::Tensor mask_nms_bool=torch::logical_and(heatmap, (tensor_semi.squeeze(0).squeeze(0) == heatmap_nms));

    // const torch::Tensor mask_nms=heatmap_bool.to(torch::kDouble);
    
#ifdef BENCHMARK
    // heatmap_benchmark.~Benchmark();
    // std::cout<<"mask_nms size: "<<mask_nms.sizes()<<std::endl;
#endif

#ifdef BENCHMARK
    Benchmark keypoint_benchmark("keypoint");
#endif


    int64_t height = binary_tensor.size(2); // 2 is the index for height
    int64_t width = binary_tensor.size(3); // 3 is the index for width
    // auto indices = torch::nonzero(mask_nms);
    // torch::Tensor indices = torch::empty({static_cast<int64_t>(keypoints.size()), 2}, torch::kInt64);
    // for (size_t i = 0; i < keypoints.size(); ++i) {
    //     if (keypoints[i].pt.x >= 0 && keypoints[i].pt.x < width && keypoints[i].pt.y >= 0 && keypoints[i].pt.y < height){
    //         indices[i][0] = static_cast<int64_t>(keypoints[i].pt.y); // Row index
    //         indices[i][1] = static_cast<int64_t>(keypoints[i].pt.x); // Column index
    //     }
        
    // }
    std::vector<int64_t> valid_indices;
    valid_indices.reserve(keypoints.size() * 2);  // Pre-reserve memory

    std::vector<cv::KeyPoint> valid_keypoints;
    valid_keypoints.reserve(keypoints.size());  // Reserve memory for valid keypoints

    for (const auto& kp : keypoints) {
        if (kp.pt.x >= 0 && kp.pt.x < width && kp.pt.y >= 0 && kp.pt.y < height) {
            valid_indices.push_back(static_cast<int64_t>(kp.pt.y));
            valid_indices.push_back(static_cast<int64_t>(kp.pt.x));
            valid_keypoints.push_back(kp);
        }
    }

    auto indices = torch::from_blob(valid_indices.data(), {static_cast<int64_t>(valid_indices.size() / 2), 2}, torch::kInt64).clone();

    // // Ensure indices are on the same device as tensor_feat
    indices = indices.to(tensor_feat.device(), torch::kLong);

    auto batch_indices = torch::zeros({indices.size(0), 1}, indices.options()).to(torch::kLong);  // Batch dimension index (all 0)
    auto channel_indices = torch::arange(256, torch::kLong).view({1, 256}).expand({indices.size(0), 256}); // Channel dimension indices

    // Repeat spatial indices for each channel
    auto y_indices = indices.select(1, 0).unsqueeze(1).expand({indices.size(0), 256});
    auto x_indices = indices.select(1, 1).unsqueeze(1).expand({indices.size(0), 256});

    // Gather the descriptors using the prepared indices
    // auto desc = tensor_feat.index({batch_indices, channel_indices, y_indices, x_indices});

    // // Remove unnecessary dimensions
    // desc = desc.squeeze(0).squeeze(1).to(torch::kDouble).cpu();  // Remove the batch dimension since it's always 1
    auto desc_binary=binary_tensor.index({batch_indices, channel_indices, y_indices, x_indices});
    desc_binary = desc_binary.squeeze(0).squeeze(1).to(torch::kUInt8).cpu();
    


    // // Normalize the descriptorss
    //desc = desc / torch::norm(desc, 2, 1, true); // Normalize along the channel dimension
    

#ifdef BENCHMARK
    keypoint_benchmark.~Benchmark();
    std::cout<<"indices size: "<<indices.sizes()<<std::endl;
    // std::cout << "Gathered desc size: " << desc.sizes() << std::endl;
    // std::cout<<"desc size: "<<desc.sizes()<<std::endl;
    std::cout<<"desc size: "<<desc_binary.sizes()<<std::endl;
#endif

#ifdef BENCHMARK
    Benchmark format_benchmark("format");
#endif

    // const Eigen::MatrixXd descr = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(desc.contiguous().data_ptr<double>(), desc.sizes()[0], desc.sizes()[1]);
    //const Eigen::MatrixXi descr_binary=Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(desc_binary.contiguous().data_ptr<int>(), desc_binary.sizes()[0], desc_binary.sizes()[1]);
    const Matrix8u descr_binary = Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(desc_binary.contiguous().data_ptr<uint8_t>(), desc_binary.sizes()[0], desc_binary.sizes()[1]);
    //std::cout<<descr_binary<<std::endl;
    // auto mask_nms_transposed = mask_nms.transpose(0, 1);

    
    // auto indices_coord = torch::nonzero(mask_nms);
    // auto indices_coord = torch::nonzero(mask_nms_transposed);
    // First, ensure the tensor is on the CPU and is contiguous
    // auto indices_contiguous = indices_coord.to(torch::kCPU).contiguous();
    auto indices_contiguous = indices.to(torch::kCPU).contiguous();

    // Convert the indices from int64_t to double
    auto indices_double = indices_contiguous.to(torch::kDouble);

    // Now map this correctly converted tensor to an Eigen matrix
    Eigen::MatrixX2d coord = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>>(
        indices_double.data_ptr<double>(),
        indices_double.size(0),
        indices_double.size(1)
    );

     // Swap columns 0 and 1
    coord.col(0).swap(coord.col(1));

    // const Eigen::Matrix2Xd coord = coord_transposed;
    // Print the modified matrix
    // std::cout << "Modified coord matrix (columns swapped):\n" << coord << std::endl;

#ifdef BENCHMARK
    format_benchmark.~Benchmark();
    // std::cout << "Number of rows in descr: " << descr.rows() << std::endl;
    // std::cout << "Number of columns in descr: " << descr.cols() << std::endl;
    std::cout << "Number of rows in descr: " << descr_binary.rows() << std::endl;
    std::cout << "Number of columns in descr: " << descr_binary.cols() << std::endl;
#endif

#ifdef BENCHMARK
    total_benchmark.~Benchmark();
#endif

    return {coord, descr_binary, valid_keypoints};
}

void Extractor::setScoreMin(float min_score)
{
    this->point_min_score = min_score;
}
