#ifndef _FASTDETECT_H_
#define _FASTDETECT_H_
#include "KeyPoint.h"
#include "TypeDef.h"
#include <opencv2/opencv.hpp>

void computeAngle(const cv::Mat &image, VecKeyPoint &keypoints); // 计算特征点方向
VecKeyPoint fastDetect(const cv::Mat &image, double percent);  // 提取Oriented FAST

#endif