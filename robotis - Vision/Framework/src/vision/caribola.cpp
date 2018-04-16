
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace std;
using namespace cv;

Mat imgHSV, imgThresholded, imgThresholdedC;

int bright, contras, saturasi, ga, expo;
int iLowH = 55;
int iHighH = 79;

int iLowS = 130;
int iHighS = 255;

int iLowV = 60;
int iHighV = 250;

int minrad = 32;
int maxrad = 159;

int ilowC = 100;
int maxlowC = 100;
int ratio = 3;

void Threshold(Mat src){

    flip(src, src, 0);
    cvtColor(src, imgHSV, CV_RGB2HSV); //Convert the captured frame from RGB to HSV

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    GaussianBlur( imgThresholded, imgThresholded, Size(3,3), 0 , 0);

    cv::Canny(imgThresholded, imgThresholdedC, ilowC, ilowC*ratio, 3);

}

void CariBola(Mat imgThresholded, Mat src){

    vector<Vec3f> circles;

    HoughCircles(imgThresholded, circles, HOUGH_GRADIENT, 1, 10,
                 100, 30, minrad, maxrad // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        koordinat.X = circles[0];
        koordinat.Y = circles[1];

        circle( src, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
        circle( src, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
    }

}
