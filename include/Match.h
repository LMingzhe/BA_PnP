#ifndef _MATCH_H_
#define _MATCH_H_
#include <Eigen/Core>

class Match
{
private:
    int id1_;
    int id2_;
    int distance_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Match();
    Match(int id1, int id2, int distance): id1_(id1), id2_(id2), distance_(distance) {}
    ~Match();

    void setID1(int id1);
    void setID2(int id2);
    void setDistance(int distance);
    int getID1();
    int getID2();
    int getDistance();
    Eigen::Vector2d get2Pid();
};

#endif