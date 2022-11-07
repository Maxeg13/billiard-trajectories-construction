//
// Created by chibi on 23.10.2022.
//

#ifndef BILLIARD_BALL_H
#define BILLIARD_BALL_H
#include <iostream>
using namespace std;
extern uint64 len_min;
extern float main_rads;
extern float rads_per_length;
extern Mat src;
struct Ball { // and ellipse, though
public:
    Ball():centre(0), rad(0) {}
    Ball(Vec2f c, float r) :centre(c), rad(r) {
        phiEll = getEllipseAlpha();
    }
    float getLeanY() {
        //  rads_per_length /2.8 is for canyon, /3.4 for samsung
        return(fabs(sin(  main_rads + rads_per_length * (centre.y - src.size().height/2 ))));
    }
    float getLeanX() {
        return(fabs(sin(  rads_per_length * (centre.x - src.size().width/2 ))));
    }
    float getX(float phi) {
        return rad * cos(phi);
    }
    MyPoint getTripod() {
        MyPoint hor = getHorizont();
        return {src.size().width/2.f, hor.y + 3.14f/2/rads_per_length};
    }
    MyPoint getHorizont() {
        return {src.size().width/2.f, src.size().height/2 -main_rads/rads_per_length};
    }
    MyPoint toAffine(MyPoint p) {
        MyPoint hor = getHorizont();
        MyPoint tripod = getTripod();
        float lean = fabs(cos(rads_per_length * sqrt(MyPoint(centre).sub(tripod).l2())));
//        cout<<getLeanY()<< endl;
        p = p.multInDir(lean, MyPoint(centre).sub(tripod));
        return p;
    }
    MyPoint fromAffine(MyPoint p) {
        MyPoint hor = getHorizont();
        MyPoint tripod = getTripod();
        float lean = fabs(cos(rads_per_length * sqrt(MyPoint(centre).sub(tripod).l2())));
//        cout<<getLeanY()<< endl;
        p = p.multInDir(1/(lean+0.00001), MyPoint(centre).sub(tripod));
        return p;
    }
    Mat getAffine() {
        Point2f imgCentre{50,50};
        MyPoint p1(0, 100);
        p1 = p1.rotate(rads_per_length * (centre.x - src.size().width/2 ));
        MyPoint p2(p1.rotate(3.14/2));
        p2 = p2.rotate(rads_per_length * (centre.x - src.size().width/2 ));
        MyPoint ap1(toAffine(p1).mult(rad/80));
        MyPoint ap2(toAffine(p2).mult(rad/80));
        Point2f src[]= {imgCentre, p1.toCV()+imgCentre, p2.toCV()+imgCentre};
        Point2f place = centre;
        place += {- 0.5f * rad, -0.3f*rad + rad * sqrt(1 - getLeanY()*getLeanY())};
        Point2f dst[]= { place, ap1.toCV() + place, ap2.toCV() + place};
        return getAffineTransform(src, dst);
    }
    float getEllipseAlpha() {
        MyPoint elTang = MyPoint(centre).sub(getTripod());
        elTang = elTang.norm();
        elTang = fromAffine(elTang);
        elTang = elTang.rotate(3.14/2);
        elTang = toAffine(elTang);
        float phi = elTang.getAngle({1., 0.});
        return phi;
    }
    float getY(float phi) {
        return rad * sin(phi) * getLeanY();
    }

    float getLean(float phi) {
        return(sqrt(cos(phi) * cos(phi) + getLeanY() * getLeanY() * sin(phi) * sin(phi)));
    }

    Point2f findNearestCentre(MyPoint origin, MyPoint direction) {
        Point2f cen;
        len_min = 10000000;
        for(float phi = -3.14; phi < 3.14; phi += 0.002) {
            MyPoint tmp{2 * getX(phi), 2 * getY(phi), 0};
            //
            tmp = tmp.rotate(phiEll);
            tmp.setAdd(MyPoint{centre});
            tmp.setSub(origin);
            uint64 len = tmp.l2();
            if( direction.mult(tmp.norm()).l2() < 0.000002 ) {
                if( len < len_min) {
//                    cout<<"len_min: "<< len_min<<"\tlen: "<<len<<"\t\tphi: "<<phi<<endl;
                    len_min = len;
//                    cout<<"now len_min: "<< len_min<<endl;
                    cen.x = tmp.x;
                    cen.y = tmp.y;
                }
            }
        }
        cen.x += origin.x;
        cen.y += origin.y;
        return cen;
    }

    MyPoint getTangentDir(MyPoint dir) {
        float tmp;
        tmp = dir.x;
        dir.x = -dir.y;
        dir.y = getLeanY() * getLeanY() * tmp;

        //
        dir = dir.rotate(phiEll);
        return dir;
    }

    bool isSame(const Ball& b) {
        float tolerance = 7;
        auto p = b.centre-centre;
        return ((sqrt(p.dot(p)) < tolerance)&&
                (fabs(rad-b.rad) < tolerance));
    }

//private:
    Point2f centre;
    float rad;
    float phiEll;
};

Ball operator+(const Ball& b1, const Ball& b2) {
    Ball res;
    res.centre = b2.centre + b1.centre;
    res.rad = b1.rad + b2.rad;
    return res;
}


struct Test {
    MyPoint origin;
    MyPoint dir;
    MyPoint ortho;
    float getLeanX() {
        return(fabs(sin(  rads_per_length * (origin.x - src.size().width/2 ))));
    }
    float getLeanY() {
        //  rads_per_length /2.8 is for canyon, /3.4 for samsung
        return(fabs(sin(  main_rads + rads_per_length * (origin.y - src.size().height/2 ))));
    }
    MyPoint toAffine(MyPoint p) {
        MyPoint hor{src.size().width/2.f, src.size().height/2 -main_rads/rads_per_length};
        MyPoint tripod{src.size().width/2.f, hor.y + 3.14f/2/rads_per_length};
        float lean = fabs(cos(rads_per_length * sqrt(origin.sub(tripod).l2())));
//        cout<<getLeanY()<< endl;
        p = p.multInDir(lean, origin.sub(tripod));
        return p;
    }
    MyPoint toTable(MyPoint p) {
        MyPoint hor{src.size().width/2.f, src.size().height/2 -main_rads/rads_per_length};
        MyPoint tripod{src.size().width/2.f, hor.y + 3.14f/2/rads_per_length};
        float lean = fabs(cos(rads_per_length * sqrt(origin.sub(tripod).l2())));
//        cout<<getLeanY()<< endl;
        p = p.multInDir(1/(lean+0.00001), origin.sub(tripod));
        return p;
    }
    MyPoint computeOrtho() {
        // ortho begin
        ortho = dir.rotate(3.14 / 2);
        ortho = toAffine(ortho);

//        Ball b{origin.toCV(),20};
//        ortho = b.getTangentDir(dir);
        return ortho;
    }
};


#endif //BILLIARD_BALL_H
