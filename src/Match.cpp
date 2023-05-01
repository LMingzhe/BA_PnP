#include "Match.h"

void Match::setID1(int id1)
{
    id1_ = id1;
}

void Match::setID2(int id2)
{
    id2_ = id2;
}

void Match::setDistance(int distance)
{
    distance_ = distance;
}

int Match::getID1()
{
    return id1_;
}

int Match::getID2()
{
    return id2_;
}

int Match::getDistance()
{
    return distance_;
}

Eigen::Vector2d Match::get2Pid()
{
    Eigen::Vector2d id;
    id << id1_, id2_;
    return id;
}

Match::~Match()
{
    
}