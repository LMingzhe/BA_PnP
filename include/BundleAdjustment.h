#ifndef _BUNDLEADJUSTMENT_H_
#define _BUNDLEADJUSTMENT_H_
#include <sophus/se3.hpp>
#include "TypeDef.h"
#include <opencv2/opencv.hpp>

void BundleAdjustment(
    const VecVector3d &point_3d,
    const VecVector2d &point_2d,
    const cv::Mat &K,
    Sophus::SE3d &pose
);

#endif