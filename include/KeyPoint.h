#ifndef _KeyPoint_H_
#define _KeyPoint_H_
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class KeyPoint
{
private:
    int u_, v_;
    int id_;
    int num_; // 周围一圈亮度差别大的点的个数，用来筛选，防止角点过于集中
    double angle_; // 旋转角度，角度制

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    KeyPoint();
    KeyPoint(int u, int v, int id, int num):u_(u), v_(v), id_(id), num_(num) {}
    ~KeyPoint();

    void setU(int u);
    void setV(int v);
    void setID(int id);
    void setNum(int num);
    void setAngle(double angle);
    int getID();
    int getNum();
    double getAngle();
    Eigen::Vector2d getPointCoordi();
};

#endif

