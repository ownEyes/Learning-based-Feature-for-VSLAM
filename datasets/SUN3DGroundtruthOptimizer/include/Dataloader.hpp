#ifndef DATALOADER_HPP
#define DATALOADER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <dirent.h>
#include <cerrno>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <png++/png.hpp>
#include <jpeglib.h>

const int kImageRows = 480;
const int kImageCols = 640;
const int kSampleFactor = 30;
const int kImageChannels = 3;
const int kFileNameLength = 24;
const int kTimeStampPos = 8;
const int kTimeStampLength = 12;

typedef unsigned char uchar;
typedef unsigned short ushort;

struct _cam_k
{
  float fx;
  float fy;
  float cx;
  float cy;
};

extern _cam_k cam_K;

typedef struct _extrinsic
{
  float R[3][3];
  float t[3];
} extrinsic;

void GetLocalFileNames(const std::string &dir, std::vector<std::string> *file_list);

// bool GetDepthData(const std::string &file_name, std::vector<ushort> &data);

// bool GetImageData(const std::string &file_name, std::vector<uchar> &data);

cv::Mat GetImage(const std::string &file_name);

cv::Mat GetDepth(const std::string &file_name);

void GetExtrinsicData(const std::string &file_name, int size, std::vector<extrinsic> *poses);

int GetTimeStamp(const std::string &file_name);

void AssignDepthList(std::vector<std::string> image_list, std::vector<std::string> *depth_list);

#endif // DATALOADER_HPP