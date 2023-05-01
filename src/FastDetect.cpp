#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>
#include "TypeDef.h"
#include "KeyPoint.h"

inline void computeAngle(const cv::Mat &image, VecKeyPoint &keypoints); // 计算旋转角度
inline bool isFAST_quick(const cv::Mat &image, int u, int v, double percent);  // 用于快速判断当前像素是否为角点(1, 5, 9, 13)
inline int isFAST(const cv::Mat &image, int u, int v, double percent, int *mask);  // 根据FAST-12定义判断是否为角点
const double pi = 3.1415926;


VecKeyPoint fastDetect(const cv::Mat &image, double percent)
{
    std::cout << "====================start fastDetect...====================" << std::endl;
    VecKeyPoint keypoints_;
    int keyPoint_num = 0; 

    int mask[16*2] = {
        -3,0, // 1
        -3,1,
        -2,2,
        -1,3,
        0,3,  // 5
        1,3,
        2,2,
        3,1,
        3,0,  // 9
        3,-1,
        2,-2,
        1,-3,
        0,-3,  // 13
        -1,-3,
        -2,-2,
        -3,-1
    };

    // int bigger, smaller, other;
    // std::cout << "Coordinates of keypoints (u, v) : " << std::endl;
    for (int v = 20; v < image.rows - 20; v++)
    {
        for (int u = 20; u < image.cols - 20; u++)  // 排除图像边缘像素点
        {
            // bigger = smaller = other = 0;
            // 快速判断有没有可能是角点，加速算法
            if (!isFAST_quick(image, u, v, percent))  
            {
                continue;
            }

            // 根据FAST-12定义判断是否为角
            int result = isFAST(image, u, v, percent, mask);
            if (result != 0)  
            {
                // std::cout << "(" << u << ", " << v << ")" << std::endl;
                keyPoint_num += 1;
                KeyPoint keypoint_(u, v, keyPoint_num, result);
                keypoints_.push_back(keypoint_);
            }
        }
    }

    computeAngle(image, keypoints_);
    // std::cout << image.size() << std::endl;
    std::cout << "total keypoints : " << keypoints_.size() << std::endl;
    std::cout << "====================fastDetect is over====================" << std::endl;
    return keypoints_;

}

// 用于快速判断当前像素是否为角点(1, 5, 9, 13)
inline bool isFAST_quick(const cv::Mat &image, int u, int v, double percent)
{
    int bigger, smaller, other;
    bigger = smaller = other = 0;

    std::vector<uchar> temp;
    uchar point_1 = image.at<uchar>(v - 3, u);
    uchar point_5 = image.at<uchar>(v, u + 3);
    uchar point_9 = image.at<uchar>(v + 3, u);
    uchar point_13 = image.at<uchar>(v, u - 3);
    temp.push_back(point_1);
    temp.push_back(point_5);
    temp.push_back(point_9);
    temp.push_back(point_13);

    for (uchar value : temp)
    {
        if (image.at<uchar>(v, u) - value > percent * image.at<uchar>(v, u))
        {
            bigger++;
        }
        else if (image.at<uchar>(v, u) - value < -percent * image.at<uchar>(v, u))
        {
            smaller++;
        }
        else
        {
            other++;
        }
    }

    if (bigger >= 3 || smaller >=3)
    {
        return true;
    }
    else
    {
        return false;
    }  
}

// 根据FAST-12定义判断是否为角点
inline int isFAST(const cv::Mat &image, int u, int v, double percent, int *mask)
{
    int bigger, smaller, other;
    bigger = smaller = other = 0;
    
    int pre = -1; // 用于记录前一个像素点是大于T_high 还是 小于 T_smaller，大于为1，小于为0，初始化为-1
    for (int k = 0; k < 16; k++)
    {
        if (bigger >= 12)
        {
            return bigger;
        }
        else if (smaller >= 12)
        {
            return smaller;
        }

        if (image.at<uchar>(v, u) - image.at<uchar>(v + mask[2*k], u + mask[2*k + 1]) > percent * image.at<uchar>(v, u))
        {
            if (pre == -1 || pre == 0)
            {
                pre = 1;
                smaller = 0;
            }
            bigger++;
        }
        else if (image.at<uchar>(v, u) - image.at<uchar>(v + mask[2*k], u + mask[2*k + 1]) < -percent * image.at<uchar>(v, u))
        {
            if (pre == -1 || pre == 1)
            {
                pre = 0;
                bigger = 0;
            }
            smaller++;
        }
        else
        {
            pre = -1;
            bigger = smaller = 0;
            other++;
        }
    }

    return 0;
}

inline void computeAngle(const cv::Mat &image, VecKeyPoint &keypoints)
{
    std::cout << "====================start computeAngle...====================" << std::endl;
    // std::cout << "(u, v)'s angle is :" << std::endl;

    int half_patch_size = 8; // 取图像块为 16x16
    for (auto &kp : keypoints)
    {
        kp.setAngle(0);
        int u = kp.getPointCoordi()(0);
        int v = kp.getPointCoordi()(1);
        
        // 边界判断，由于在角点提取 fastDetect 中已排除了边缘点，这里不用判断。
        // int cols = image.cols, rows = image.rows;
        // if (u - half_patch_size < 0 || u + half_patch_size - 1 > cols - 1 || v - half_patch_size < 0 || v + half_patch_size - 1 > rows - 1)
        // {
        //     continue;
        // }

        // 计算重心
        int m10 = 0;
        int m01 = 0;
        // 取窗口大小为16x16
        for (int i = u - half_patch_size; i < u + half_patch_size; i++)
        {
            for (int j = v - half_patch_size; j < v + half_patch_size; j++)
            {
                m10 += (i - u) * image.at<uchar>(j, i);
                m01 += (j - v) * image.at<uchar>(j, i);
            }
        }
        // 计算角度
        double rad = std::atan2(m01, m10);
        double angle = (180.0 / pi) * rad;
        // std::cout << "(" << u << ", " << v <<"): " << angle << std::endl;
        kp.setAngle(angle);
    }

    std::cout << "====================computeAngle is over====================" << std::endl;
}