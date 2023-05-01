#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "KeyPoint.h"

typedef std::vector<bool> DesType; // 描述子Vec
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d; // 3D坐标点容器
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d; // 2D坐标点容器
typedef std::vector<KeyPoint> VecKeyPoint;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

#endif