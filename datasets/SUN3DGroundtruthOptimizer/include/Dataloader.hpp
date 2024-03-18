#ifndef DATALOADER_HPP
#define DATALOADER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <dirent.h>
#include <cerrno>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Geometry> 
// #include <png++/png.hpp>
// #include <jpeglib.h>

const int kImageRows = 480;
const int kImageCols = 640;
const int kSampleFactor = 30;
const int kImageChannels = 3;
const int kFileNameLength = 24;
const int kTimeStampPos = 8;
const int kTimeStampLength = 12;

typedef unsigned char uchar;
typedef unsigned short ushort;

// Define the struct using cv::Mat
typedef struct _cam_k
{
    cv::Mat K; // Matrix to represent intrinsic camera parameters

    // Constructor to initialize the matrix with fx, fy, cx, cy
    _cam_k(float fx, float fy, float cx, float cy) : K(cv::Mat::zeros(3, 3, CV_32F)) // Initialize a 3x3 matrix filled with zeros
    {
        K.at<float>(0, 0) = fx; // Set fx at position (0, 0)
        K.at<float>(1, 1) = fy; // Set fy at position (1, 1)
        K.at<float>(0, 2) = cx; // Set cx at position (0, 2)
        K.at<float>(1, 2) = cy; // Set cy at position (1, 2)
        K.at<float>(2, 2) = 1;  // Set the scale factor for homogeneous coordinates
    }

} cam_K;


typedef struct _extrinsic {
    // Default constructor
    _extrinsic() : t(Eigen::Vector3f::Zero()), translation{0.0f, 0.0f, 0.0f} {
        Eigen::Matrix3f R_mat = Eigen::Matrix3f::Identity();
        set_rotation_matrix(R_mat);
    }

    // Constructor to initialize from a data array
    explicit _extrinsic(float* data_addr) {
        set_from(data_addr);
    }

    // Populate the struct from a data array
    void set_from(float* data_addr) {
        Eigen::Map<Eigen::Matrix3f> R_mat(data_addr);
        set_rotation_matrix(R_mat);
        Eigen::Map<Eigen::Vector3f> t_vec(data_addr + 9);
        t = t_vec;
        std::copy(data_addr + 9, data_addr + 12, translation);
    }

    // Store the struct's data in a data array
    void set_to(float* data_addr) {
        Eigen::Map<Eigen::Matrix3f> R_mat(data_addr);
        R_mat = rotation.matrix();
        Eigen::Map<Eigen::Vector3f> t_vec(data_addr + 9);
        t_vec = t;
        std::copy(t.data(), t.data() + 3, data_addr + 9);
    }

    // Method to update the rotation matrix from an Eigen matrix
    void set_rotation_matrix(const Eigen::Matrix3f& R_mat) {
        rotation = Sophus::SO3f(R_mat);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = R_mat(i, j);
            }
        }
    }

    // Method to get an SE3 representation of the pose
    Sophus::SE3f get_se3() const {
        return Sophus::SE3f(rotation, t);
    }

    // Method to set the pose from an SE3 representation
    void set_from_se3(const Sophus::SE3f& se3) {
        rotation = se3.so3();
        t = se3.translation();
        set_rotation_matrix(rotation.matrix());
        std::copy(t.data(), t.data() + 3, translation);
    }

    Sophus::SO3f rotation;  // Rotation represented as an SO3 object
    Eigen::Vector3f t;  // Translation vector using Eigen
    float R[3][3];  // Rotation matrix as a simple array
    float translation[3];  // Translation vector as a simple array
} extrinsic;

void GetLocalFileNames(const std::string &dir, std::vector<std::string> *file_list);

// bool GetDepthData(const std::string &file_name, std::vector<ushort> &data);

// bool GetImageData(const std::string &file_name, std::vector<uchar> &data);

cv::Mat GetImage(const std::string &file_name);

cv::Mat GetDepth(const std::string &file_name);

void GetExtrinsicData(const std::string &file_name, int size, std::vector<extrinsic> *poses);

void GetCameraK(const std::string &file_name, cv::Mat &K);

int GetTimeStamp(const std::string &file_name);

void AssignDepthList(std::vector<std::string> image_list, std::vector<std::string> *depth_list);

#endif // DATALOADER_HPP