//https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
// don forget to cvt colors if different channels size making bitwise operations
#include <math.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <thread>
#include <queue>
#include "MyPoint.h"
#include "Ball.h"
#include <iostream>
using namespace cv;
using namespace std;
int mouse_x;
int mouse_y;
float scale = 1.3;
float rads_per_length = 0.0018/scale; // 0.0025/3.4
int hor_correction = 0;
bool draw_is = true;
bool draw_src_is = true;
Mat src;
Mat interaction_mask;
Mat interaction_scene;
thread gravity_thread;
float hue_rads;
float main_rads;

Test test{MyPoint(200,200), MyPoint(100,100), MyPoint(100,70)};

vector<Ball> balls;

uint64 len_min = 10000000;

void graviTask();

Ball cue_ball;
vector<deque<Ball>> correctors;
void trajectoriesFunc();
int param1 = 140, param2 = 27;
int test_val;
int* mouse_interface = &param1;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        draw_is = !draw_is;
        cout << "draw is: " << draw_is << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout<<"mouse interface, ";
        static int cnt = 0;
        cnt++;
        switch(cnt%4) {
            case 0:
                cout<<"param1"<<endl;
                mouse_interface = &param1;
                break;
            case 1:
                cout<<"param2"<<endl;
                mouse_interface = &param2;
                break;
            case 2:
                cout<<"horiz correction"<<endl;
                mouse_interface = &hor_correction;
                break;
            case 3:
                cout<<"test val"<<endl;
                mouse_interface = &test_val;
                break;
        }
    }
    else if(event == EVENT_MBUTTONDBLCLK) {
        test.origin = {(float)mouse_x, (float)mouse_y};
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        if(draw_is) {
            mouse_x = x;
            mouse_y = y;
        }
    }
    else if( event == EVENT_MOUSEWHEEL) {
        int sign_ = (flags>0)?(1):(-1);
        *mouse_interface += 2 * sign_;
        cout << "flags: " << flags << endl;
        cout << "val: " << *mouse_interface << endl;
    }
}

//#define TEST
//#define DEBUG
//#define SYMBOL
const char *filename = "billiard.jpg";
Mat symbol;
Mat symbol_inter;
string ip_addr = //"192.168.162.219:8080";
        "192.168.0.100:8080";
int main()
{
    symbol = imread( "symbol.png" );
    symbol.convertTo(symbol, -1, 1, -85);

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
        resize(src,src,Size(src.size().width/2.6 * scale, src.size().height/2.6 * scale));
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
        // src is formed

        resize(symbol,symbol_inter,src.size());

        Mat gray;
        cvtColor(src, gray, COLOR_BGR2GRAY);
        medianBlur(gray, gray, 5);
        vector<Vec3f> circles;
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1.,
                     gray.rows / 16,  // change this value to detect circles with different distances to each other
                     param1, param2, 7, 50 // change the last two parameters 180 50
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
            if(deq.size() == 0) {
                same = false;
            }

            if(same) {
                deq.push_back(balls[i]);
            }
            else { // bad situation, one needs to be accurate
                if(deq.size() == 0) {
                    deq.push_back(balls[i]);
                }
                else {
                    correctors[i].pop_front();
                }
            }

            if(correctors[i].size()>8)
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
//            cout<<ball.getLeanY()<<endl;
            Point2f center = ball.centre;
            if (ball.rad > rad_max) {
                rad_max = ball.rad;
                cue_ball = ball;
            }

            int radius = ball.rad;

#ifdef SYMBOL
            matrix = ball.getAffine();
            warpAffine(symbol, symbol_inter, matrix, src.size());
            bitwise_or(src, symbol_inter, src);
#endif

            // circle center
            if(draw_src_is) {
                circle(src, center, 1, Scalar(0, 100, 100), 1, LINE_AA);

                // circle outline
                circle(src, center, radius, Scalar(255, 0, 255), 2, LINE_AA);
                putText(src, to_string(ind), (Point) center + Point(-radius, -radius), {}, 0.5, Scalar(255, 0, 255), 2,
                        LINE_AA);

                // real ellipse
                float phi = ball.getEllipseAlpha();
                ellipse(src, center, {radius, static_cast<int>(radius * ball.getLeanY())}, phi/3.14*180, 0,
                        180, Scalar(255, 0, 255), 2);
            }
            ind++;
        }

        //horizon
        line(src, Point(0, src.size().height/2 - main_rads / rads_per_length),
             Point(src.size().width, src.size().height/2 - main_rads / rads_per_length), Scalar(255, 255, 255), 1);
        trajectoriesFunc();

        if(waitKey(1) == 'g')
        {
            cout<<"g!"<<endl;
            draw_src_is = ! draw_src_is;
        }
    }
    //    serial->close();
    return EXIT_SUCCESS;
}

void trajectoriesFunc() {

    interaction_scene = src.clone();
    if(draw_is)
        interaction_mask = Mat::zeros(interaction_scene.size(), interaction_scene.type());
    // cursor

    circle(interaction_scene, Point(mouse_x, mouse_y), 1, Scalar(255, 255, 255), 2, LINE_AA);

    // for  cue
    MyPoint p(Point2f(mouse_x, mouse_y) - cue_ball.centre);
    p = p.norm().mult(cue_ball.rad);
    if (p.y > 0) {
        p = p.mult(cue_ball.getLean(p.getAtan()));
    }
    Point2f indent{p.x, p.y};

    MyPoint cueDir{Point2f(mouse_x, mouse_y) - cue_ball.centre};
    cueDir = cueDir.norm();

    // draw cue
    if (draw_is) {
        line(interaction_mask, MyPoint(cue_ball.centre + indent).add(cueDir.mult(35)).toCV(),
             cue_ball.centre + indent, Scalar(255, 210, 210), 3);
    }


    for (auto &ball: balls) {
        if (ball.centre == cue_ball.centre) {
            continue;
        }

        Point2f nearest = ball.findNearestCentre(cue_ball.centre,
                                                 MyPoint(Point2f(mouse_x, mouse_y) - cue_ball.centre).norm());
        if (nearest == cue_ball.centre)
            continue;
        // here we go
        MyPoint halfDir = (MyPoint(nearest).add(ball.centre)).mult(0.5).sub(cue_ball.centre);

        if (MyPoint(ball.centre - cue_ball.centre).multScal(cueDir) < 0)
            cueDir.setMult(-1);

        // ellipse
        if (draw_is) {
            float phi = ball.getEllipseAlpha();
            ellipse(interaction_mask, nearest, {(int) ball.rad, static_cast<int>(ball.rad * ball.getLeanY())}, phi/3.14*180, 0,
                    180, Scalar(100, 255, 100), 1);
        }
        int radius = ball.rad;

        if (draw_is) {
            circle(interaction_mask, nearest, radius, Scalar(200, 255, 100), 2, LINE_AA);
        }

        // touching point
        MyPoint tp(nearest);
        tp.setAdd(ball.centre);
        tp.setMult(0.5);
        circle(interaction_scene, tp.toCV(), 1, Scalar(255, 0, 255), 2, LINE_AA);
        if(draw_is) {
//            circle(interaction_mask, tp.toCV(), 1, Scalar(0, 255, 255), 2, LINE_AA);
        }

        // and final direction
        MyPoint objDir(ball.centre - nearest);
        objDir = objDir.norm();
        objDir = objDir.mult(1000);

        if (draw_is)
            line(interaction_mask, ball.centre, ball.centre + Point2f{objDir.x, objDir.y}, Scalar(255, 0, 255), 2);

        // svoj shar, otskok
        MyPoint dirTangAlter = ball.fromAffine(objDir);
        dirTangAlter = dirTangAlter.rotate(3.14/2);
        dirTangAlter = ball.toAffine(dirTangAlter);
        MyPoint dirTang = ball.getTangentDir(objDir);
        float cueSign = 1;
        if (objDir.mult(cueDir).z < 0)
            cueSign = -1;

        dirTangAlter = dirTangAlter.mult(-cueSign);
        dirTang = dirTang.mult(cueSign);

        // tang 1
//        if (draw_is)
//            line(interaction_mask, nearest, nearest + Point2f{dirTang.x, dirTang.y}, Scalar(200, 255, 100), 2);

        // tang 2
        if (draw_is)
            line(interaction_mask, nearest, nearest + Point2f{dirTangAlter.x, dirTangAlter.y}, Scalar(70, 255, 200), 2);

        ///////////////////
        // cut
        int cut_rad = 27;
        cueDir = ball.fromAffine(cueDir);
        objDir = ball.fromAffine(objDir);
        float angle = cueDir.getAngle(objDir);
        float cut = sin(angle);
        int shift = -cut * cut_rad * 2;
        bool is_golden = ((fabs(cut) <= 0.5) && (fabs(cut) > 0.35));

        putText(interaction_scene, to_string(int(angle*180/3.14))+" gr",
                Point(interaction_scene.size().width - 80, 120), {}, 0.5, Scalar(255, 0, 255), 2,
                LINE_AA);

        if (draw_is) {
            circle(interaction_mask, Point(interaction_scene.size().width - 60, 60),
                   cut_rad, Scalar(0, 255, 255), -2, LINE_AA);
            line(interaction_mask, Point(interaction_scene.size().width - 60, 60 - cut_rad),
                 Point(interaction_scene.size().width - 60, 60 + cut_rad), Scalar(0, 0, 0), 1);
            circle(interaction_mask, Point(interaction_scene.size().width - 60 + shift, 60),
                   cut_rad, Scalar(255, 0, 255), -2, LINE_AA);
        }
        cueDir = ball.toAffine(cueDir);
        objDir = ball.toAffine(objDir);

        // golden trajectories
        if (draw_is && is_golden) {
            MyPoint goldenDir1(ball.fromAffine(halfDir)), goldenDir2(ball.fromAffine(halfDir));
            goldenDir1 = goldenDir1.rotate((-31) * cueSign / (180 / 3.14));
            goldenDir2 = goldenDir2.rotate((-39) * cueSign / (180 / 3.14));
            goldenDir1.setMult(700);
            goldenDir2.setMult(700);
            goldenDir1 = ball.toAffine(goldenDir1);
            goldenDir2 = ball.toAffine(goldenDir2);
            Mat mask = Mat::zeros(interaction_scene.size(), interaction_scene.type());
            vector<Point> poly{nearest, nearest + Point2f{goldenDir1.x, goldenDir1.y},
                                 nearest + Point2f{goldenDir2.x, goldenDir2.y}};
            fillPoly(mask, poly, Scalar(0, 120, 100));
            bitwise_or( mask, interaction_mask, interaction_mask);
        }
    }

    // and test finally
#ifdef TEST
    test.dir = MyPoint(100,70);
    test.dir = test.dir.rotate(test_val/30.);
    test.computeOrtho();
    test.dir = test.toAffine(test.dir);

    line(interaction_mask, test.origin.toCV(),
         (test.dir.add(test.origin)).toCV(), Scalar(100, 255, 255), 1);

    line(interaction_mask, test.origin.toCV(),
         (test.ortho.add(test.origin)).toCV(), Scalar(100, 255, 255), 1);
#endif

    bitwise_or(interaction_mask, interaction_scene, interaction_scene);

    imshow("billiard", interaction_scene);
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
