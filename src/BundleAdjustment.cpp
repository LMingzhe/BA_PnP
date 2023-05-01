#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include "BundleAdjustment.h"
#include "TypeDef.h"

void BundleAdjustment(
    const VecVector3d &point_3d,
    const VecVector2d &point_2d,
    const cv::Mat &K,
    Sophus::SE3d &pose
)
{
    std::cout << "====================start BundleAdjustment...====================" << std::endl;

    int iterations = 100;
    double cost = 0, lastcost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; iter++)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < point_3d.size(); i++)
        {
            Eigen::Vector2d e;  // 误差
            Eigen::Vector2d p2d_p = point_2d[i];  // 像素坐标2d
            Eigen::Vector3d p3d_w = point_3d[i];  // 世界坐标3d
            Eigen::Vector3d p3d_c = pose * p3d_w;  // 世界坐标转化为相机坐标
            double inv_z = 1.0 / p3d_c[2];
            double inv_z2 = inv_z * inv_z;

            e(0) = p2d_p[0] - (fx * p3d_c[0] / p3d_c[2] + cx);
            e(1) = p2d_p[1] - (fy * p3d_c[1] / p3d_c[2] + cy);

            cost += e.transpose() * e;
            Eigen::Matrix<double, 2, 6> J; //2*6的雅克比矩阵，即误差相对于位姿求导，通过导数可以知道有了误差以后我们应该往哪个方向去优化
            J << -fx * inv_z,
                0,
                fx * p3d_c[0] * inv_z2,
                fx * p3d_c[0] * p3d_c[1] * inv_z2,
                -fx - fx * p3d_c[0] * p3d_c[0] * inv_z2,
                fx * p3d_c[1] * inv_z,
                0,
                -fy * inv_z,
                fy * p3d_c[1] * inv_z2,
                fy + fy * p3d_c[1] * p3d_c[1] * inv_z2,
                -fy * p3d_c[0] * p3d_c[1] * inv_z2,
                -fy * p3d_c[0] * inv_z;

            H += J.transpose() * J;  // 高斯牛顿 H*dx=b
            b += -J.transpose() * e;
        }

        Vector6d dx;   
        dx = H.ldlt().solve(b);  // Cholesky分解求dx

        if (isnan(dx[0]))
        {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        if (iter > 0 && cost >= lastcost)  
        {
            // cost is increase, update is not good
            std::cout << cost << ", last cost:" << lastcost << std::endl;
            break;
        }

        // update estimation
        pose = Sophus::SE3d::exp(dx) * pose;

        lastcost = cost;

        std::cout << "iteration " << iter << " cost=" << std::cout.precision(12) << cost << std::endl;
    }

    std::cout << "estimated pose by GN-BA: \n" << pose.matrix() << std::endl;

    std::cout << "====================BundleAdjustment is over====================" << std::endl;
    
}