// The code does simple visual odometry. In current version it is not 
// calculating correct position of the robot, since more information is
// required for that (information from single camera is not enough).
// For demonstration properties it shows the direction of optical flow,
// which enables to track camera movements in relative values. 

// Vahe Taamazyan, Skoltech, 2015
// For Robotics class 

#include <stdio.h>
#include <stdlib.h> 
#include <iostream>
#include <math.h>
#include <ctime>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    // Camera Calibration Data
    Mat K = (Mat_<double>(3,3) << 237, 0, 326, 
        0, 236, 272, 
        0, 0, 1); 

    Mat pos = (Mat_<double>(3,1) << 0,0,0);
    Mat R0 = (Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1);
    Mat image, gray, prevGray;
    Mat map (600, 600, CV_32F); 
    vector<Point2f> points[2];
    Point2f t0;
    t0.x = 300;
    t0.y = 300;
    int kol = 0;
    VideoCapture cap (1);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 500;
    bool needToInit = true;

    // Sparse Optical Flow between two consequent images is calculated 
    for (;;)
    {
        kol++;
        cout << kol << endl;
        Mat frame;
        cap >> frame;
        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        if ( needToInit )
        {
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            needToInit = false;
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
            size_t i, k, k1;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( !status[i] )
                    continue;
                points[1][k++] = points[1][i];
            }
            for( i = k1 = 0; i < points[0].size(); i++ )
            {
                if( !status[i] )
                    continue;
                points[0][k1++] = points[0][i];
            }
            for (i = 0; i< points[0].size(); i++)
            {
                line (image, points[0][i], points[1][i], Scalar(0,255,0), 2);
            }
            points[1].resize(k);
            points[0].resize(k1);
            
        }
        imshow("My Flow", image);   
        if ( !points[1].empty() && !points[0].empty() )
        {
            Mat F = findFundamentalMat(points[0], points[1], CV_FM_RANSAC, 2,0.99);
            std::vector<double> v1,v2;
            v1.resize(3);
            v2.resize(3);
            v1[0] = points[0][5].x;
            v1[1] = points[0][5].y;
            v1[2] = 1;
            v2[0] = points[1][5].x;
            v2[1] = points[1][5].y;
            v2[2] = 1;
            Mat p1 = Mat(v1);
            Mat p2 = Mat(v2);
            Mat A = p2.t() * F * p1;                        
            std::cout << "A = " << std::endl << " " << A << std::endl << std::endl;
            std::cout << "F = " << std::endl << " " << F << std::endl << std::endl;
            Mat E = K.t() * F * K;
            std::cout << "E = " << std::endl << " " << E << std::endl << std::endl;
            SVD svd(E);
            Matx33d W(0,-1,0,
                    1,0,0,
                    0,0,1);
            Matx33d Z(0,1,0
                -1,0,0,
                0,0,1);
            Matx33d Z1(0,1,0
                -1,0,0,
                0,0,0);
            Mat WW = Mat::diag(svd.w);
            Mat R = svd.u * Mat(W) * svd.vt;
            Mat t1 = svd.u.col(2);

            Mat tx = svd.vt.t()*Mat(W)*WW*svd.vt;
            Mat t(3, 1, CV_64F);
            t.at<double>(0,0) = tx.at<double>(2,1);
            t.at<double>(1,0) = tx.at<double>(0,2);
            t.at<double>(2,0) = tx.at<double>(1,0);
            if (kol >= 2)
            {
                Mat t2 = cv::abs(t1);
                std::cout << "T1 = " << std::endl << " " << t1 << std::endl << std::endl;
                std::cout << "T2 = " << std::endl << " " << t2 << std::endl << std::endl;
                double maxval;
                minMaxLoc(t2, NULL, &maxval, NULL, NULL);
                cout << maxval << endl;
                t1 = t1/maxval;
                t1 = t1*10;
            }
            Point2f t3;
            t3.x = 0;
            t3.y = 0;
            for (size_t i=0; i<points[0].size(); i++)
            {
                t3.x = t3.x + (points[1][i].x - points[0][i].x);
                t3.y = t3.y - (points[1][i].y - points[0][i].y);
            }
            t3.x = t3.x/points[0].size();
            t3.y = t3.y/points[0].size();
            std::cout << "T3 = " << std::endl << " " << t3 << std::endl << std::endl;
            t0.x = t0.x + t3.x;
            t0.y = t0.y + t3.y;
            cout << t0 << endl;
            circle( map, t0, 3, Scalar(255,255,255), -1, 8);
            std::cout << "R = " << std::endl << " " << R << std::endl << std::endl;
            std::cout << "T = " << std::endl << " " << t << std::endl << std::endl;
            

        }
        imshow("Camera Position", map);
        if (points[1].size() < 50)
        {
            points[0].clear();
            needToInit = true;
        }
        else
        {
            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);
        }

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
    }

    return 0;
}
