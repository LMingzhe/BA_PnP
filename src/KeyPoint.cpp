#include "TypeDef.h"
#include "KeyPoint.h"

void KeyPoint::setU(int u)
{
    u_ = u;
}

void KeyPoint::setV(int v)
{
    v_ = v;
}

void KeyPoint::setID(int id)
{
    id_ = id;
}

void KeyPoint::setAngle(double angle)
{
    angle_ = angle;
}

void KeyPoint::setNum(int num)
{
    num_ = num;
}

int KeyPoint::getID()
{
    return id_;
}

int KeyPoint::getNum()
{
    return num_;
}

double KeyPoint::getAngle()
{
    return angle_;
}

Eigen::Vector2d KeyPoint::getPointCoordi() 
{
    Eigen::Vector2d point;
    point << u_, v_;
    return point;
}

KeyPoint::~KeyPoint()
{

}
