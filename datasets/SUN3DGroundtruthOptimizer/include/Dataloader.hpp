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
#include <exception>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Geometry>
// #include <png++/png.hpp>
// #include <jpeglib.h>

void handleException(const std::exception &e, const char *file, int line);

#define TRY_CATCH_BLOCK(code)                   \
    try                                         \
    {                                           \
        code                                    \
    }                                           \
    catch (const std::exception &e)             \
    {                                           \
        handleException(e, __FILE__, __LINE__); \
    }

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

typedef struct _extrinsic
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

    Sophus::SO3d rotation; // Rotation represented as an SO3 object
    Eigen::Vector3d t;     // Translation vector using Eigen
    Eigen::Matrix3d R_mat; // Directly store the rotation matrix
    double R[3][3];        // Rotation matrix as a simple array
    double translation[3]; // Translation vector as a simple array

    // Default constructor
    _extrinsic() : t(Eigen::Vector3d::Zero()), R_mat(Eigen::Matrix3d::Identity())
    {
        updateArrays();
        rotation = Sophus::SO3d(R_mat);
    }

    // Update the simple array representations from the Eigen types
    void updateArrays()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R[i][j] = R_mat(i, j);
            }
            translation[i] = t[i];
        }
    }

    // Method to get an SE3 representation of the pose
    Sophus::SE3d get_se3() const
    {
        return Sophus::SE3d(rotation, t);
    }

    // Method to set the pose from an SE3 representation
    void set_from_se3(const Sophus::SE3d &se3)
    {
        rotation = se3.so3();
        t = se3.translation();
        R_mat = se3.rotationMatrix();
        updateArrays();
    }

    // Ensure the rotation matrix is orthogonal
    void makeOrthogonal()
    {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R_mat = svd.matrixU() * svd.matrixV().transpose();
    }

    // Method to set the pose from buffer representation
    void update_from_arrays()
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R_mat(i, j) = R[i][j];
            }
        }
        // Make sure R is orthogonal by applying SVD
        makeOrthogonal();

        TRY_CATCH_BLOCK({
            rotation = Sophus::SO3d(R_mat);
        })

        t = Eigen::Map<Eigen::Vector3d>(translation);
    }
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