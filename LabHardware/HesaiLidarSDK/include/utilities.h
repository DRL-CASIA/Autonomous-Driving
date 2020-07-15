#ifndef PANDORA_UTILITY_H
#define PANDORA_UTILITY_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

void yuvToRgb(const int &iY, const int &iU, const int &iV, int &iR, int &iG, int &iB);
void yuv400ToCvmat(cv::Mat &dst, void *pYUV400, int nWidth, int nHeight, int bitDepth);
void yuv422ToRgb24(unsigned char *uyvy422, unsigned char *rgb24, int width, int height);
void yuv422ToCvmat(cv::Mat &dst, void *pYUV422, int nWidth, int nHeight, int bitDepth);
bool loadCameraIntrinsics(const std::string &filename, std::vector<cv::Mat> &intrinsicKList, std::vector<cv::Mat> &intrinsicDList);
int decompressJpeg(unsigned char *jpgBuffer, unsigned long jpgSize, unsigned char * &bmp, unsigned long& bmpSize);

#endif