#include <iostream>
#include <string>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "TypeDef.h"
#include "KeyPoint.h"
#include "FastDetect.h"
#include "GetBrief.h"
#include "Match.h"
#include "BFMatch.h"
#include "BundleAdjustment.h"

// 像素坐标转化为相机坐标
Eigen::Vector2d pixel2cam(KeyPoint &kp_, cv::Mat &K_);

std::string first_file = "../data/1.png";
std::string second_file = "../data/2.png";
std::string depth_file = "../data/1_depth.png";

int main(int argc, char **argv)
{
    cv::Mat img1_rgb = cv::imread(first_file, 1);
    cv::Mat img2_rgb = cv::imread(second_file, 1);
    cv::Mat img1 = cv::imread(first_file, 0);
    cv::Mat img2 = cv::imread(second_file, 0);
    assert(img1.data != nullptr && img2.data != nullptr);
    cv::Mat img_depth = cv::imread(depth_file, CV_LOAD_IMAGE_UNCHANGED);

    // 提取带旋转角度的FAST角点, percent可以更改
    VecKeyPoint keypoints_1 = fastDetect(img1, 0.8);
    VecKeyPoint keypoints_2 = fastDetect(img2, 0.8);

    // 画出特征点
    for (int i = 0; i < keypoints_1.size(); i++)
    {
        cv::Point p;
        p.x = keypoints_1[i].getPointCoordi()(0);
        p.y = keypoints_1[i].getPointCoordi()(1);
        cv::circle(img1_rgb, p, 2, cv::Scalar(255, 255, 0), -1);
    }

    for(int i = 0; i < keypoints_2.size(); i++)
    {
        cv::Point p;
        p.x = keypoints_2[i].getPointCoordi()(0);
        p.y = keypoints_2[i].getPointCoordi()(1);
        cv::circle(img2_rgb, p, 2, cv::Scalar(255, 255, 0), -1);
    }

    cv::imshow("img1_rgb", img1_rgb);
    cv::imshow("img2_rgb", img2_rgb);
    cv::waitKey(0);

    // 计算特征点的描述子
    std::vector<DesType> desc1, desc2;
    desc1 = getBrief(img1, keypoints_1);
    desc2 = getBrief(img2, keypoints_2);

    // 匹配描述子，matches包括所有匹配，matches_good是匹配成功的
    std::vector<Match> matches, matches_good;
    matches = bfMatch(desc1, desc2);
    std::vector<double> dd_vec;

    // 计算最大最小距离
    auto min_max = std::minmax_element(matches.begin(), matches.end(), 
                [](Match &m1, Match &m2) { return m1.getDistance() < m2.getDistance();});
    double min_dist = min_max.first->getDistance();
    double max_dist = min_max.second->getDistance();
    std::cout << "Min distance: " << min_dist << std::endl;
    std::cout << "Max distance: " << max_dist << std::endl;
    
    for (int i = 0; i < matches.size(); i++)
    {
        int d_u = keypoints_1[matches[i].getID1()].getPointCoordi()(0);
        int d_v = keypoints_1[matches[i].getID1()].getPointCoordi()(1);

        int d2_u = keypoints_2[matches[i].getID2()].getPointCoordi()(0);
        int d2_v = keypoints_2[matches[i].getID2()].getPointCoordi()(1);

        int distance = matches[i].getDistance();

        unsigned short depth = img_depth.at<unsigned short>(d_v, d_u);
        // 去掉图像中深度为0的点和距离太小的点
        if (depth == 0 || distance < std::max(2 * min_dist, 30.0))
        {
            continue;
        }

        double dd = depth / 5000.0;
        std::cout << "depth: " << depth << ", " << "dd : " << dd << std::endl;

        dd_vec.push_back(dd);
        matches_good.push_back(matches[i]);
    }
    std::cout << "一共找到了" << matches_good.size() << "组匹配点" << std::endl;

    // 绘制匹配成功图像
    cv::Mat combine;
    cv::hconcat(img1_rgb, img2_rgb, combine);
    // 绘制匹配成功点对连线
    for (int i = 0; i < matches_good.size(); i++)
    {   
        cv:: Point p1 = cv::Point(keypoints_1[matches_good[i].getID1()].getPointCoordi()(0),
                                keypoints_1[matches_good[i].getID1()].getPointCoordi()(1));
        
        cv:: Point p2 = cv::Point(keypoints_2[matches_good[i].getID2()].getPointCoordi()(0) + 640,
                                keypoints_2[matches_good[i].getID2()].getPointCoordi()(1));

        cv::line(combine, p1, p2, cv::Scalar(200, 0, 0), 1);
    }

    // BA优化位姿
    VecVector3d points_3d;
    VecVector2d points_2d;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    Sophus::SE3d pose;
    for (int i = 0; i < dd_vec.size(); i++) 
    {
        double dd = dd_vec[i];
        int id1 = matches_good[i].getID1();
        int id2 = matches_good[i].getID2();
        KeyPoint pixel = keypoints_1[id1];
        Eigen::Vector2d cam = pixel2cam(pixel, K);

        points_3d.push_back(Eigen::Vector3d(cam(0)*dd, cam(1)*dd, dd));
        points_2d.push_back(Eigen::Vector2d(keypoints_2[id2].getPointCoordi()(0),
                                                keypoints_2[id2].getPointCoordi()(1)));
    }
    // for (auto p3d:points_3d)
    // {
    //     std::cout << p3d(0) << ", "<< p3d(1) << ", " << p3d(2) << std::endl;
    // }
    // for (auto p2d:points_2d)
    // {
    //     std::cout << p2d(0) << ", " << p2d(1) << std::endl;
    // }
    BundleAdjustment(points_3d, points_2d, K, pose);

    cv::imshow("combine", combine);
    cv::waitKey(0);

    return 0;
}

Eigen::Vector2d pixel2cam(KeyPoint &kp_, cv::Mat &K_)
{
    int u = kp_.getPointCoordi()(0);
    int v = kp_.getPointCoordi()(1);
    double fx = K_.at<double>(0, 0);
    double fy = K_.at<double>(1, 1);
    double cx = K_.at<double>(0, 2);
    double cy = K_.at<double>(1, 2);

    Eigen::Vector2d vec_;
    vec_(0) = ((u - cx)/fx);
    vec_(1) = ((v - cy)/fy);
    return vec_;
}