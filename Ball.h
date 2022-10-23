//
// Created by chibi on 23.10.2022.
//

#ifndef BILLIARD_BALL_H
#define BILLIARD_BALL_H
extern uint64 len_min;
extern float main_rads;
extern Mat src;
extern int test_val;
struct Ball { // and ellipse, though
public:
    Ball():centre(0), rad(0) {}
    Ball(Vec2f c, float r) :centre(c), rad(r) {}
    float getLeanY() {
        //  rads_per_length /2.8 is for canyon, /3.4 for samsung
        return(fabs(sin(  main_rads + rads_per_length/3.4 * (centre.y - src.size().height/2 + test_val))));
    }
    float getLeanX() {
        return(fabs(sin(  rads_per_length * (centre.x - src.size().width/2 ))));
    }
    float getX(float phi) {
        return rad * cos(phi);
    }
    MyPoint fromAffine(MyPoint x) {
//        x.x/=getLeanX();
        x.y/=getLeanY();
        return x;
    }
    MyPoint toAffine(MyPoint x) {
//        x.x*=getLeanX();
        x.y*=getLeanY();
        return x;
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
    static constexpr float rads_per_length = 0.0025;
};

Ball operator+(const Ball& b1, const Ball& b2) {
    Ball res;
    res.centre = b2.centre + b1.centre;
    res.rad = b1.rad + b2.rad;
    return res;
}

#endif //BILLIARD_BALL_H
