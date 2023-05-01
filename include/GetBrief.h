#ifndef _GETBRIEF_H_
#define _GETBRIEF_H_
#include <opencv2/opencv.hpp>
#include "TypeDef.h"
#include "KeyPoint.h"

std::vector<DesType> getBrief(const cv::Mat &image, VecKeyPoint &keypoints_);  // 计算ORB描述子

#endif
