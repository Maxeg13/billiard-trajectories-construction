#ifndef POINT_H
#define POINT_H

#include <math.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

class MyPoint
{
public:
    MyPoint(Point2f p): x(p.x), y(p.y) {

    }
    MyPoint(float x, float y, float z): x(x), y(y), z(z) {

    }

    MyPoint() = default;

    MyPoint(const MyPoint&) = default;

    MyPoint sub(MyPoint p) {
        MyPoint _p;
        _p.x = x-p.x;
        _p.y = y-p.y;
        _p.z = z - p.z;
        return _p;
    }

    float l2() {
        return x*x+y*y+z*z;
    }

    MyPoint norm() {
        float l = sqrt(l2());
        MyPoint y(*this);
        y.setMult(1/l);
        return y;
    }

    MyPoint add(MyPoint p) {
        MyPoint _p;
        _p.x = x+p.x;
        _p.y = y+p.y;
        _p.z = z + p.z;
        return _p;
    }

    float getAtan() {
        return atan(y/x);
    }

    MyPoint setAdd(MyPoint p) {
        x+=p.x;
        y+=p.y;
        z+=p.z;
        return *this;
    }

    MyPoint setSub(MyPoint p) {
        x-=p.x;
        y-=p.y;
        z-=p.z;
        return *this;
    }

    MyPoint setMult(float _x) {
        x*=_x;
        y*=_x;
        z*=_x;
        return *this;
    }

    float multScal(MyPoint p) {
        return x*p.x+y*p.y+z*p.z;
    }

    MyPoint mult(MyPoint p) {
        MyPoint p1(*this);
        p1.x = y*p.z - z*p.y;
        p1.y = z*p.x - x*p.z;
        p1.z = x*p.y - y*p.x;
        return p1;
    }

    MyPoint mult(float _x) {
        MyPoint _p(*this);
        _p.x*=_x;
        _p.y*=_x;
        _p.z*=_x;
        return _p;
    }

    float getAngle(MyPoint p) {
        float m = mult(p).z;
        return m/(sqrt(l2()) * sqrt(p.l2()));
    }

    float x;
    float y;
    float z;

};
#endif