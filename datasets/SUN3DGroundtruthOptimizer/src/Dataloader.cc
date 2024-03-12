#include <Dataloader.hpp>

using namespace std;

_cam_k cam_K;

void GetLocalFileNames(const string &dir, vector<string> *file_list)
{
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(dir.c_str())) == NULL)
  {
    cout << "Error(" << errno << ") opening " << dir << endl;
    exit(EXIT_FAILURE); // Terminate the program
  }

  while ((dirp = readdir(dp)) != NULL)
  {
    file_list->push_back(dir + string(dirp->d_name));
  }
  closedir(dp);

  sort(file_list->begin(), file_list->end());
  file_list->erase(file_list->begin()); //.
  file_list->erase(file_list->begin()); //..
}

cv::Mat GetImage(const string &file_name)
{
  return cv::imread(file_name, cv::IMREAD_COLOR);
}

cv::Mat GetDepth(const string &file_name)
{
  cv::Mat depthData = cv::imread(file_name, cv::IMREAD_UNCHANGED);
  if (depthData.type() == CV_16UC1)
  {
    for (int i = 0; i < depthData.rows; ++i)
    {
      for (int j = 0; j < depthData.cols; ++j)
      {
        ushort &pixel = depthData.at<ushort>(i, j);
        pixel = (pixel << 13 | pixel >> 3);
      }
    }
  }
  return depthData;
}

// bool GetDepthData(const std::string &file_name, std::vector<ushort> &data) {
//     png::image<png::gray_pixel_16> img(file_name);

//     data.resize(kImageRows * kImageCols);

//     for (int i = 0; i < kImageRows; ++i) {
//         for (int j = 0; j < kImageCols; ++j) {
//             ushort s = img.get_pixel(j, i);
//             data[i * kImageCols + j] = (s << 13 | s >> 3);
//         }
//     }

//     return true;
// }

// bool GetImageData(const string &file_name, vector<uchar> &data) {
//   struct jpeg_decompress_struct cinfo;
//   struct jpeg_error_mgr jerr;

//   FILE *infile = fopen(file_name.c_str(), "rb");
//   unsigned long location = 0;

//   if (!infile) {
//     std::cerr << "Error opening jpeg file " << file_name << "!" << std::endl;
//     return false;
//   }
//   cinfo.err = jpeg_std_error(&jerr);
//   jpeg_create_decompress(&cinfo);
//   jpeg_stdio_src(&cinfo, infile);
//   jpeg_read_header(&cinfo, TRUE);
//   jpeg_start_decompress(&cinfo);

//   std::vector<uchar> raw_image(cinfo.output_width * cinfo.output_height * cinfo.num_components);
//   std::vector<uchar> row_buffer(cinfo.output_width * cinfo.num_components);

//    while (cinfo.output_scanline < cinfo.image_height) {
//         uchar* row_address = &row_buffer[0];
//         jpeg_read_scanlines(&cinfo, &row_address, 1);
//         std::copy(row_buffer.begin(), row_buffer.end(), raw_image.begin() + cinfo.output_scanline * row_buffer.size() - row_buffer.size());
//     }

//   data.resize(cinfo.output_width * cinfo.output_height * kImageChannels);
//   std::copy(raw_image.begin(), raw_image.end(), data.begin());

//   jpeg_finish_decompress(&cinfo);
//   jpeg_destroy_decompress(&cinfo);
//   fclose(infile);
//   return true;
// }

void GetExtrinsicData(const string &file_name, int size,
                      vector<extrinsic> *poses)
{
  FILE *fp = fopen(file_name.c_str(), "r");

  for (int i = 0; i < size; ++i)
  {
    extrinsic m;
    for (int d = 0; d < 3; ++d)
    {
      int iret;
      iret = fscanf(fp, "%f", &m.R[d][0]);
      iret = fscanf(fp, "%f", &m.R[d][1]);
      iret = fscanf(fp, "%f", &m.R[d][2]);
      iret = fscanf(fp, "%f", &m.t[d]);
    }
    poses->push_back(m);
  }

  fclose(fp);
}

int GetTimeStamp(const string &file_name)
{
  return atoi(file_name.substr(
                           file_name.size() - kFileNameLength + kTimeStampPos,
                           kTimeStampLength)
                  .c_str());
}

void AssignDepthList(vector<string> image_list, vector<string> *depth_list)
{
  vector<string> depth_temp;
  depth_temp.swap(*depth_list);
  depth_list->clear();
  depth_list->reserve(image_list.size());

  int idx = 0;
  int depth_time = GetTimeStamp(depth_temp[idx]);
  int time_low = depth_time;

  for (unsigned int i = 0; i < image_list.size(); ++i)
  {
    int image_time = GetTimeStamp(image_list[i]);

    while (depth_time < image_time)
    {
      if (idx == depth_temp.size() - 1)
        break;

      time_low = depth_time;
      depth_time = GetTimeStamp(depth_temp[++idx]);
    }

    if (idx == 0 && depth_time > image_time)
    {
      depth_list->push_back(depth_temp[idx]);
      continue;
    }

    if (abs(image_time - time_low) < abs(depth_time - image_time))
    {
      depth_list->push_back(depth_temp[idx - 1]);
    }
    else
    {
      depth_list->push_back(depth_temp[idx]);
    }
  }
}
