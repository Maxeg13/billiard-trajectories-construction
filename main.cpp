//https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
// don forget to cvt colors if different channels size making bitwise operations
#include <math.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <thread>
#include <queue>
#include "MyPoint.h"
#include <iostream>
using namespace cv;
using namespace std;
int mouse_x;
int mouse_y;
int horiz;
bool draw_is = true;
Mat src;
Mat interaction_scene;
thread gravity_thread;
float hue_rads;
float main_rads;
struct Ball;
vector<Ball> balls;

uint64 len_min = 10000000;

void graviTask();

struct Ball { // and ellipse, though
public:
    Ball():centre(0), rad(0) {}
    Ball(Vec2f c, float r) :centre(c), rad(r) {}
    float getLeanY() {
        // /2.8 is for canyon
        return(fabs(sin(  main_rads+ rads_per_length/3.4 * (centre.y - src.size().height/2))));
    }
    float getX(float phi) {
        return rad * cos(phi);
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

Ball cue_ball;
vector<deque<Ball>> correctors;
void trajectoriesFunc();

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        draw_is = !draw_is;
        cout << "draw is: " << draw_is << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        mouse_x = x;
        mouse_y = y;
    }
}

#define DEBUG
const char *filename = "billiard.jpg";
string ip_addr = //"192.168.40.178:8080";
        "192.168.0.100:8080";
int main()
{
    namedWindow( "billiard");
#ifndef DEBUG // usual
    gravity_thread = thread(graviTask);
    gravity_thread.detach();

    VideoCapture cap("http://" +ip_addr +"//video?x.mjpeg&req_fps=10");
    while(!cap.isOpened());
#else //debug

#endif

    setMouseCallback("billiard", CallBackFunc, NULL);

    /////////
    for(;;) {
#ifdef DEBUG
        src = imread( filename );
#else
        cap >> src;
        if(src.empty()) {

            cap.open("http://" +ip_addr +"//video?x.mjpeg&req_fps=10");
            continue;
        }
        resize(src,src,Size(src.size().width/2.6, src.size().height/2.6));
        auto matrix = getRotationMatrix2D(
                Point2f{(float)src.size().width/2, (float)src.size().height/2}, hue_rads*180/3.14, 1);
        warpAffine(src, src, matrix, src.size());
#endif

        if (src.empty()) {
            cout << "empty\n";
            printf(" Error opening image\n");
            printf(" Program Arguments: [image_name -- default %s] \n", filename);
            return EXIT_FAILURE;
        }
        Mat gray;
        cvtColor(src, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 7);
        vector<Vec3f> circles;
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1.,
                     gray.rows / 16,  // change this value to detect circles with different distances to each other
                     140, 40, 7, 50 // change the last two parameters 180 50
                // (min_radius & max_radius) to detect larger circles
        );

        balls.clear();
        for (const auto &circle: circles) {
            balls.emplace_back(Vec2f(circle[0], circle[1]), circle[2]);
        }
        sort(balls.begin(), balls.end(), [](Ball a, Ball b){return a.centre.y>b.centre.y;});

        //smoothed motion
        correctors.resize(balls.size());
        for (int i = 0; i<balls.size(); i++) {
            auto& deq = correctors[i];
            bool same = true;
            for(auto& ex: deq) {
                same = same && balls[i].isSame(ex);

            }
            if(same) {
                deq.push_back(balls[i]);
            } else {
                cout<<"clear"<<endl;
                deq.clear();
                deq.push_back(balls[i]);
            }
            if(correctors[i].size()>7)
                correctors[i].pop_front();

            // correction itself
            Ball acc;
            balls[i] = Ball();
            for(Ball b: deq) {
                acc = acc + b;
            }

            if(deq.size()) {
                balls[i].rad = acc.rad / deq.size();
                balls[i].centre.x = acc.centre.x / deq.size();
                balls[i].centre.y = acc.centre.y / deq.size();
            }
        }

        // draw primary stuff
        int rad_max = 0;
        int ind = 0;
        for (auto &ball: balls) {

            Point2f center = ball.centre;
            if (ball.rad > rad_max) {
                rad_max = ball.rad;
                cue_ball = ball;
            }

            int radius = ball.rad;

            // circle center
            circle(src, center, 1, Scalar(0, 100, 100), 2, LINE_AA);

            // circle outline
            circle(src, center, radius, Scalar(255, 0, 255), 2, LINE_AA);
            putText(src, to_string(ind), (Point)center + Point(-radius, -radius), {}, 0.5, Scalar(255, 0, 255), 2,
                    LINE_AA);

            // real ellipse
            ellipse(src, center, {radius, static_cast<int>(radius * ball.getLeanY())}, 0, 0,
                    180, Scalar(255, 0, 255),2);

            ind++;
        }

        trajectoriesFunc();

        waitKey(42);
    }
    //    serial->close();
    return EXIT_SUCCESS;
}

void trajectoriesFunc() {

    interaction_scene = src.clone();
    // cursor

    circle( interaction_scene, Point(mouse_x,mouse_y), 1, Scalar(0,100,100), 2, LINE_AA);

    // for  cue
    MyPoint p(Point2f(mouse_x,mouse_y) - cue_ball.centre);
    p = p.norm().mult(cue_ball.rad);
    if(p.y > 0) {
        p = p.mult(cue_ball.getLean(p.getAtan()));
    }
    Point2f intend{ p.x, p.y};

    // draw cue
    if(draw_is)
        line(interaction_scene, Point(mouse_x,mouse_y), cue_ball.centre + intend , Scalar(255, 210, 210), 5);

    MyPoint cueDir{Point2f(mouse_x,mouse_y) - cue_ball.centre};
    cueDir = cueDir.norm();

    for(auto& ball: balls) {
        if(ball.centre == cue_ball.centre) {
            continue;
        }

        Point2f nearest = ball.findNearestCentre(cue_ball.centre, MyPoint(Point2f(mouse_x, mouse_y) - cue_ball.centre).norm() );
        if(nearest == cue_ball.centre )
            continue;

        if(draw_is)
            ellipse(interaction_scene, nearest, {(int)ball.rad, static_cast<int>(ball.rad * ball.getLeanY())}, 0, 0,
                    180, Scalar(100,255,100), 2);

        int radius = ball.rad;

        if(draw_is)
            circle( interaction_scene, nearest, radius, Scalar(200,255,100), 2, LINE_AA);

        // and final direction
        MyPoint dir(ball.centre - nearest);
        dir = dir.norm();
        dir = dir.mult(1000);

        if(draw_is)
            line(interaction_scene, ball.centre, ball.centre + Point2f{dir.x, dir.y} , Scalar(255,0,255), 2);

        // svoj shar, otskok
        MyPoint dirTang = ball.getTangentDir(dir);
        float _sign = -1;
        if( dir.mult(cueDir).z < 0)
            _sign = 1;

        dirTang = dirTang.mult(_sign);

        if(draw_is)
            line(interaction_scene, nearest, nearest + Point2f{dirTang.x, dirTang.y} , Scalar(200,255,100), 2);

        // rezka
        {
            int rad = 27;
            cueDir.y /= ball.getLeanY();
            dir.y /= ball.getLeanY();

            int shift = sin(cueDir.getAngle(dir)) * rad * 2;

            if(draw_is) {
                circle(interaction_scene, Point(interaction_scene.size().width - 60, 60),
                       rad, Scalar(0, 255, 255), -2, LINE_AA);
                line(interaction_scene, Point(interaction_scene.size().width - 60, 60 - rad) ,
                     Point(interaction_scene.size().width - 60, 60 + rad),Scalar(0,0,0), 1);
                circle(interaction_scene, Point(interaction_scene.size().width - 60 + shift, 60),
                       rad, Scalar(255, 0, 255), -2, LINE_AA);
            }
        }
    }

    imshow("billiard", interaction_scene);
//        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
}

#include "GravityProcessing.h"
void graviTask() {
//    int8_t gx, gy,gz;
//    float leny, lenx;
//    LPCTSTR com_port = "\\\\.\\COM7";
//    DWORD COM_BAUD_RATE = CBR_115200;
//    serial = new Serial;
//    serial->InitCOM(com_port);
//
//    int cnt = 0;
//    for(;;) {
//        cnt++;
//        serial->write('s');
//        string incoming = serial->ReadCOM();
//        if (incoming.size() == 3) {
//            cout << incoming.size() << endl;
//            gx = (uint8_t) incoming.c_str()[0];
//            gy = (uint8_t) incoming.c_str()[1];
//            gz = (uint8_t) incoming.c_str()[2];
//            cout<<"step: "<<cnt<<endl;
////            cout << (int) gx << "\t" << (int) gy << "\t" << (int) gz << endl;
//            leny = (float)gy*gy + (float)gz*gz;
//            leny = sqrt(leny);
//            lenx = (float)gx*gx + (float)gz*gz;
//            lenx = sqrt(lenx);
//            main_rads = asin(gz/leny)+0.05;
//            cout << "main angle: "<< main_rads * 180/3.14 << "\t";
//            cout << "side angle: "<< acos(gz/lenx) * 180/3.14 << endl;
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(150));
//    }
//    UDPServer();
curl_main();
}
