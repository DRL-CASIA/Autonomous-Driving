
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <boost/lexical_cast.hpp>
#include "yaml-cpp/yaml.h"
#include <jpeglib.h>
#include "utilities.h"


void yuv400ToCvmat(cv::Mat &dst, void *pYUV400, int nWidth, int nHeight, int bitDepth)
{
	IplImage *imgHeaderP;

	if (!pYUV400)
	{
		return;
	}

	if (bitDepth == 8)
	{
		imgHeaderP = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	}
	else
	{
		imgHeaderP = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_16U, 1);
	}

	cvSetData(imgHeaderP, (unsigned char *)pYUV400, nWidth);
	dst = cv::cvarrToMat(imgHeaderP);
	cvReleaseImageHeader(&imgHeaderP);
}

void yuvToRgb(const int &iY, const int &iU, const int &iV, int &iR, int &iG, int &iB)
{
	assert(&iR != NULL && &iG != NULL && &iB != NULL);

	iR = iY + 1.13983 * (iV - 128);
	iG = iY - 0.39465 * (iU - 128) - 0.58060 * (iV - 128);
	iB = iY + 2.03211 * (iU - 128);

	iR = iR > 255 ? 255 : iR;
	iR = iR < 0 ? 0 : iR;

	iG = iG > 255 ? 255 : iG;
	iG = iG < 0 ? 0 : iG;

	iB = iB > 255 ? 255 : iB;
	iB = iB < 0 ? 0 : iB;
}

void yuv422ToRgb24(unsigned char *uyvy422, unsigned char *rgb24, int width, int height)
{
	int iR, iG, iB;
	int iY0, iY1, iU, iV;
	int i = 0;
	int j = 0;
	for (i = 0; i < width * height * 2; i += 4)
	{
		iU = uyvy422[i + 0];
		iY0 = uyvy422[i + 1];
		iV = uyvy422[i + 2];
		iY1 = uyvy422[i + 3];

		yuvToRgb(iY0, iU, iV, iR, iG, iB);
		// rgb24[j++] = iR;
		// rgb24[j++] = iG;
		// rgb24[j++] = iB;
		rgb24[j++] = iB;
		rgb24[j++] = iG;
		rgb24[j++] = iR;
		yuvToRgb(iY1, iU, iV, iR, iG, iB);
		// rgb24[j++] = iR;
		// rgb24[j++] = iG;
		// rgb24[j++] = iB;
		rgb24[j++] = iB;
		rgb24[j++] = iG;
		rgb24[j++] = iR;
	}
}

void yuv422ToCvmat(cv::Mat &dst, void *pYUV422, int nWidth, int nHeight, int bitDepth)
{
	if (!pYUV422)
	{
		return;
	}
	unsigned char *rgb24_buffer = new unsigned char[nWidth * nHeight * 3];
	yuv422ToRgb24((unsigned char *)pYUV422, rgb24_buffer, nWidth, nHeight);
	dst = cv::Mat(nHeight, nWidth, CV_8UC3, rgb24_buffer).clone();
	delete[] rgb24_buffer;
}

bool loadCameraIntrinsics(const std::string &filename, std::vector<cv::Mat> &intrinsicKList, std::vector<cv::Mat> &intrinsicDList)
{
	if ((access(filename.c_str(), 0)) == -1)
	{
		printf("invalid intrinsicFile\n");
		return false;
	}
	YAML::Node yn = YAML::LoadFile(filename);
	std::string cameraId;
	for (int id = 0; id < 5; ++id)
	{
		// cameraId = std::to_string(i);
		cameraId = boost::lexical_cast<std::string>(id);
		cv::Mat intrinsicK, intrinsicD;

		if (yn[cameraId]["K"].IsDefined())
		{
			intrinsicK = cv::Mat::zeros(3, 3, CV_64FC1);
			for (int i = 0; i < yn[cameraId]["K"].size(); ++i)
			{
				intrinsicK.at<double>(i) = yn[cameraId]["K"][i].as<double>();
			}
			intrinsicKList.push_back(intrinsicK);
		}
		else
		{
			printf("invalid intrinsicFile content\n");
			return false;
		}
		if (yn[cameraId]["D"].IsDefined())
		{
			intrinsicD = cv::Mat::zeros(yn[cameraId]["D"].size(), 1, CV_64FC1);
			for (int i = 0; i < yn[cameraId]["D"].size(); ++i)
			{
				intrinsicD.at<double>(i) = yn[cameraId]["D"][i].as<double>();
			}
			intrinsicDList.push_back(intrinsicD);
		}
		else
		{
			printf("invalid intrinsicFile content\n");
			return false;
		}
	}
	return true;
}


void my_output_message (j_common_ptr ptr)
{
  return;
}

void print_mem(unsigned char* mem , unsigned int size)
{
	int i =0;
	for(i = 0 ; i < size ; i++)
	{
		printf("%02x " , mem[i]);
	}
	printf("\n");
}

int decompressJpeg(unsigned char *jpgBuffer, unsigned long jpgSize, unsigned char * &bmp, unsigned long& bmpSize)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  unsigned char *bmpBuffer;
  int rowStride, width, height, pixelSize;

  cinfo.err = jpeg_std_error(&jerr);
  cinfo.err->output_message = my_output_message;
	cinfo.err->error_exit = my_output_message;

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpgBuffer, jpgSize);

  int rc = jpeg_read_header(&cinfo, TRUE);
  if (rc != 1)
  {
    return -1;
  }

  jpeg_start_decompress(&cinfo);

  width = cinfo.output_width;
  height = cinfo.output_height;
  pixelSize = cinfo.output_components;
  bmpSize = width * height * pixelSize;
  bmpBuffer = (unsigned char *)malloc(bmpSize);
  rowStride = width * pixelSize;

  while (cinfo.output_scanline < cinfo.output_height)
  {
    unsigned char *buffer_array[1];
    buffer_array[0] = bmpBuffer +
                      (cinfo.output_scanline) * rowStride;

    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }
  bmp = bmpBuffer;
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  return 0;
}