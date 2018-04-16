/*
 *   ImgProcess.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "ImgProcess.h"
#include "MotionStatus.h"
#include "Camera.h"
#include "JointData.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"

using namespace Robot;
using namespace cv;
using namespace std;

ImgProcess::ImgProcess()
{    
    hogfile = "../../../Data/cvHOGClassifier.yaml";

    hog.winSize = Size(64, 64); // Default training images size as used in paper

//    hog.setSVMDetector(descriptorVector);
    hog.load(hogfile);
    
    hitThreshold = 1.8375034;
//    hitThreshold = TRAINHOG_SVM_TO_TRAIN::getInstance()->getThreshold();   
    
    ada = 0;
    tiada = 0; 

    size.width  = Camera::WIDTH;
    size.height = Camera::HEIGHT;

    size1.width  = Camera::WIDTH;
    size1.height = Camera::HEIGHT;

    //green params
    iLowH  = 30;    iLowS  = 120;   iLowV  = 22;
    iHighH = 75;    iHighS = 255;   iHighV = 255;
    
    erodegreen = 8;
    dilategreen = 3;

    // // white params
    wLowH  = 0;     wLowS  = 0;     wLowV  = 230;
    wHighH = 255;   wHighS = 50;    wHighV = 255;

    erodewhite = 2;
    dilatewhite = 20;

    //houghcircle params
    dp = 2;
    minrad = 50;    
    maxrad = 360;
    centerthresh = 5;
    cannyhough = 50;
    mindist = 360;    

    namedWindow("Green", CV_WINDOW_AUTOSIZE);
    namedWindow("White", CV_WINDOW_AUTOSIZE);

    cvCreateTrackbar("LowH", "White", &wLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("LowS", "White", &wLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("LowV", "White", &wLowV, 255); //Value (0 - 255)

    cvCreateTrackbar("HighH", "White", &wHighH, 179);
    cvCreateTrackbar("HighS", "White", &wHighS, 255);    
    cvCreateTrackbar("HighV", "White", &wHighV, 255);

    cvCreateTrackbar("Dilate", "White", &dilatewhite, 50);
    cvCreateTrackbar("Erode", "White", &erodewhite, 50);

    cvCreateTrackbar("LowH", "Green", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("LowS", "Green", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("LowV", "Green", &iLowV, 255); //Value (0 - 255)

    cvCreateTrackbar("HighH", "Green", &iHighH, 179);    
    cvCreateTrackbar("HighS", "Green", &iHighS, 255);
    cvCreateTrackbar("HighV", "Green", &iHighV, 255);

    cvCreateTrackbar("Dilate", "Green", &dilategreen, 50);
    cvCreateTrackbar("Erode", "Green", &erodegreen, 50);
 
    cvCreateTrackbar("MinRad", "Green", &minrad, 1000); //Value (0 - 255)
    cvCreateTrackbar("MaxRad", "Green", &maxrad, 2000);

    cvCreateTrackbar("Center Thresh", "Green", &centerthresh, 500);
    cvCreateTrackbar("Canny params", "Green", &cannyhough, 500);
    cvCreateTrackbar("dp", "Green", &dp, 10);
    cvCreateTrackbar("mindist", "Green", &mindist, 1000);
}

ImgProcess::~ImgProcess()
{
}

void ImgProcess::YUVtoRGB(FrameBuffer *buf)
{
    unsigned char *yuyv, *rgb;
    int z = 0;

    yuyv = buf->m_YUVFrame->m_ImageData;
    rgb = buf->m_RGBFrame->m_ImageData;

    for(int height = 0; height < buf->m_YUVFrame->m_Height; height++)
    {
        for(int width = 0; width < buf->m_YUVFrame->m_Width; width++)
        {
            int r, g, b;
            int y, u, v;

            if(!z)
                y = yuyv[0] << 8;
            else
                y = yuyv[2] << 8;
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(rgb++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(rgb++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(rgb++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++)
            {
                z = 0;
                yuyv += 4;
            }
        }
    }
}

void ImgProcess::RGBtoHSV(FrameBuffer *buf)
{
    int ir, ig, ib, imin, imax;
    int th, ts, tv, diffvmin;

    for(int i = 0; i < buf->m_RGBFrame->m_Width*buf->m_RGBFrame->m_Height; i++)
    {
        ir = buf->m_RGBFrame->m_ImageData[3*i+0];
        ig = buf->m_RGBFrame->m_ImageData[3*i+1];
        ib = buf->m_RGBFrame->m_ImageData[3*i+2];

        if( ir > ig )
        {
            imax = ir;
            imin = ig;
        }
        else
        {
            imax = ig;
            imin = ir;
        }

        if( imax > ib ) {
            if( imin > ib ) imin = ib;
        } else imax = ib;

        tv = imax;
        diffvmin = imax - imin;

        if( (tv!=0) && (diffvmin!=0) )
        {
            ts = (255* diffvmin) / imax;
            if( tv == ir ) th = (ig-ib)*60/diffvmin;
            else if( tv == ig ) th = 120 + (ib-ir)*60/diffvmin;
            else th = 240 + (ir-ig)*60/diffvmin;
            if( th < 0 ) th += 360;
            th &= 0x0000FFFF;
        }
        else
        {
            tv = 0;
            ts = 0;
            th = 0xFFFF;
        }

        ts = ts * 100 / 255;
        tv = tv * 100 / 255;

        //buf->m_HSVFrame->m_ImageData[i]= (unsigned int)th | ((unsigned int)ts<<16) | ((unsigned int)tv<<24);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+0] = (unsigned char)(th >> 8);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+1] = (unsigned char)(th & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+2] = (unsigned char)(ts & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+3] = (unsigned char)(tv & 0xFF);
    }
}

void ImgProcess::Erosion(Image* img)
{
    int x, y;

    unsigned char* temp_img = new unsigned char[img->m_Width*img->m_Height];
    memset(temp_img, 0, img->m_Width*img->m_Height);

    for( y=1; y<(img->m_Height-1); y++ )
    {
        for( x=1; x<(img->m_Width-1); x++ )
        {
            temp_img[y*img->m_Width+x]= img->m_ImageData[(y-1)*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y-1)*img->m_Width+(x  )]
                                      & img->m_ImageData[(y-1)*img->m_Width+(x+1)]
                                      & img->m_ImageData[(y  )*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y  )*img->m_Width+(x  )]
                                      & img->m_ImageData[(y  )*img->m_Width+(x+1)]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x  )]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x+1)];
        }
    }

    memcpy(img->m_ImageData, temp_img, img->m_Width*img->m_Height);

    delete[] temp_img;
}

void ImgProcess::Erosion(Image* src, Image* dest)
{
    int x, y;

    for( y=1; y<(src->m_Height-1); y++ )
    {
        for( x=1; x<(src->m_Width-1); x++ )
        {
            dest->m_ImageData[y*src->m_Width+x]= src->m_ImageData[(y-1)*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y-1)*src->m_Width+(x  )]
                                               & src->m_ImageData[(y-1)*src->m_Width+(x+1)]
                                               & src->m_ImageData[(y  )*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y  )*src->m_Width+(x  )]
                                               & src->m_ImageData[(y  )*src->m_Width+(x+1)]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x  )]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x+1)];
        }
    }
}

void ImgProcess::Dilation(Image* img)
{
    int x, y;

    unsigned char* temp_img = new unsigned char[img->m_Width*img->m_Height];
    memset(temp_img, 0, img->m_Width*img->m_Height);

    for( y=1; y<(img->m_Height-1); y++ )
    {
        for( x=1; x<(img->m_Width-1); x++ )
        {
            temp_img[y*img->m_Width+x]= img->m_ImageData[(y-1)*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y-1)*img->m_Width+(x  )]
                                      | img->m_ImageData[(y-1)*img->m_Width+(x+1)]
                                      | img->m_ImageData[(y  )*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y  )*img->m_Width+(x  )]
                                      | img->m_ImageData[(y  )*img->m_Width+(x+1)]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x  )]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x+1)];
        }
    }

    memcpy(img->m_ImageData, temp_img, img->m_Width*img->m_Height);

    delete[] temp_img;
}

void ImgProcess::Dilation(Image* src, Image* dest)
{
    int x, y;

    for( y=1; y<(src->m_Height-1); y++ )
    {
        for( x=1; x<(src->m_Width-1); x++ )
        {
            dest->m_ImageData[y*src->m_Width+x]= src->m_ImageData[(y-1)*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y-1)*src->m_Width+(x  )]
                                               | src->m_ImageData[(y-1)*src->m_Width+(x+1)]
                                               | src->m_ImageData[(y  )*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y  )*src->m_Width+(x  )]
                                               | src->m_ImageData[(y  )*src->m_Width+(x+1)]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x  )]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x+1)];

        }
    }
}

void ImgProcess::HFlipYUV(Image* img)
{
    int sizeline = img->m_Width * 2; /* 2 bytes per pixel*/
    unsigned char *pframe;
    pframe=img->m_ImageData;
    unsigned char line[sizeline-1];/*line buffer*/
    for (int h = 0; h < img->m_Height; h++)
    {   /*line iterator*/
        for(int w = sizeline-1; w > 0; w = w - 4)
        {   /* pixel iterator */
            line[w-1]=*pframe++;
            line[w-2]=*pframe++;
            line[w-3]=*pframe++;
            line[w]=*pframe++;
        }
        memcpy(img->m_ImageData+(h*sizeline), line, sizeline); /*copy reversed line to frame buffer*/
    }
}

void ImgProcess::VFlipYUV(Image* img)
{
    int sizeline = img->m_Width * 2; /* 2 bytes per pixel */
    unsigned char line1[sizeline-1];/*line1 buffer*/
    unsigned char line2[sizeline-1];/*line2 buffer*/
    for(int h = 0; h < img->m_Height/2; h++)
    {   /*line iterator*/
        memcpy(line1,img->m_ImageData+h*sizeline,sizeline);
        memcpy(line2,img->m_ImageData+(img->m_Height-1-h)*sizeline,sizeline);

        memcpy(img->m_ImageData+h*sizeline, line2, sizeline);
        memcpy(img->m_ImageData+(img->m_Height-1-h)*sizeline, line1, sizeline);
    }
}

// ***   WEBOTS PART  *** //

void ImgProcess::BGRAtoHSV(FrameBuffer *buf)
{
    int ir, ig, ib, imin, imax;
    int th, ts, tv, diffvmin;

    for(int i = 0; i < buf->m_BGRAFrame->m_Width*buf->m_BGRAFrame->m_Height; i++)
    {
        ib = buf->m_BGRAFrame->m_ImageData[4*i+0];
        ig = buf->m_BGRAFrame->m_ImageData[4*i+1];
        ir = buf->m_BGRAFrame->m_ImageData[4*i+2];

        if( ir > ig )
        {
            imax = ir;
            imin = ig;
        }
        else
        {
            imax = ig;
            imin = ir;
        }

        if( imax > ib ) {
            if( imin > ib ) imin = ib;
        } else imax = ib;

        tv = imax;
        diffvmin = imax - imin;

        if( (tv!=0) && (diffvmin!=0) )
        {
            ts = (255* diffvmin) / imax;
            if( tv == ir ) th = (ig-ib)*60/diffvmin;
            else if( tv == ig ) th = 120 + (ib-ir)*60/diffvmin;
            else th = 240 + (ir-ig)*60/diffvmin;
            if( th < 0 ) th += 360;
            th &= 0x0000FFFF;
        }
        else
        {
            tv = 0;
            ts = 0;
            th = 0xFFFF;
        }

        ts = ts * 100 / 255;
        tv = tv * 100 / 255;

        //buf->m_HSVFrame->m_ImageData[i]= (unsigned int)th | ((unsigned int)ts<<16) | ((unsigned int)tv<<24);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+0] = (unsigned char)(th >> 8);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+1] = (unsigned char)(th & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+2] = (unsigned char)(ts & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+3] = (unsigned char)(tv & 0xFF);
    }
}


void ImgProcess::scanBolaDeket(Point2D &koordinat, Mat imgThresholded, Mat imgThresholdedW){
    Size imej = imgThresholded.size();
    bool scanIjo, scanDone;
    scanIjo = false; scanDone = false;
    int dapet, dapet2, ceklubang, ceky, x, y, dapetp;
    ceklubang = 0;
    dapet = -1000; dapet2 = 1000; dapetp = 0;

    for(y = imej.height - 160; y >= (imej.height / 2) - 45 ; y-= 10){

        for(x=20 ; x < imej.width - 40; x+=2){
            if(imgThresholded.at<uchar>(y,x) == 0){
                dapetp = 0;
                ceklubang = 0;
                for(int m = 0; m <= 10; m++){
                    if(imgThresholded.at<uchar>(y, x + m) == 0){ 
                        ceklubang++;
                    }
                    if(imgThresholdedW.at<uchar>(y * 320 / 480, (x + m)  * 320 / 480) == 255){
                        dapetp++;
                    }
                }

                if(ceklubang > 5 && dapetp > 5){
                    dapet = x;
                    ceky = y - 10;
                    dapetp = 0;
                    ceklubang = 0;
                    scanIjo = true;
                    break;
                }
            }    
        }

        if(scanIjo == true){
            for(x = imej.width - 20 ; x >= 40 ; x-=2){
                if(imgThresholded.at<uchar>(y,x) == 0){
                    dapetp = 0;
                    ceklubang = 0;
                    for(int m = 0; m <= 10; m++){
                        if(imgThresholded.at<uchar>(y,x - m) == 0){ 
                            ceklubang++;
                        }
                        if(imgThresholdedW.at<uchar>(y  * 320 / 480, (x - m) * 320 / 480) == 255){
                            dapetp++;
                        }
                    }

                    if(ceklubang > 5 && dapetp > 5){
                        dapetp = 0;
                        ceklubang = 0;
                        dapet2 = x;
                        break;
                    }
                }    
            }            
        }

        if(abs(dapet2 - dapet) > 80 && abs(dapet2 - dapet) < 320){
            koordinat.X = (dapet2 + dapet) / 2;
            koordinat.Y = ceky;
            break;
        } else{
            koordinat.X = -1;
            koordinat.Y = -1;
        }
        if(scanDone == true){
            break;
        }
    }

    // printf("\n\n========== Ceky  : %d", ceky);

    // printf("\n\n========== Dapet  : %d", dapet);
    // printf("\n\n========== Dapet2 : %d", dapet2);
    // printf("\n\n========== DapetS : %d", abs(dapet2 - dapet));
    // printf("\n\n=========== cek lubang : %d", ceklubang);
    koordinat.Y += 80;
    if(koordinat.Y > size.height){
        koordinat.Y = size.height;
    }

    // if(koordinat.X > 160){
    //     koordinat.X += 40;
    // } else koordinat.X -= 40;
    
    // if(koordinat.X > 320){
    //     koordinat.X = 320;
    // } else if(koordinat.X < -1){
    //     koordinat.X = 0;
    // }

    if(koordinat.X != -1)
    {        
        adabola = true;
    } 
    else adabola = false;


    selisih.X = abs(koordinat.X - koordinatlama.X);
    selisih.Y = abs(koordinat.Y - koordinatlama.Y);

    if(koordinat.X > -1 && koordinat.Y > -1){
        
        if(selisih.X < 100 || selisih.Y < 100){
            for(int i = 4; i >= 0; i--){
                deteksi[i] = deteksi[i-1];
            }
            if(ada < 5){
                ada++;
            }
            deteksi[0] = koordinat;
        }        

    } 
    else{
        tiada++;
    }

    if(ada > 0){
        koordinat.X = -1;
        koordinat.Y = -1;

        for(int i = 0; i < ada; i++){
            koordinat.X += deteksi[i].X;
            koordinat.Y += deteksi[i].Y;
        }
        koordinat.X /= ada;            
        koordinat.Y /= ada;                
    }


    koordinatlama = koordinat; 
    
    if(tiada == 5){
        for(int i = 0; i < 5; i++){
            deteksi[i].X = 0;
            deteksi[i].Y = 0;
        }
        koordinatsimpan = koordinat;
        tiada = 0;
        ada = 0;
    }

}

void ImgProcess::featuredetect(Mat frame, Mat imgThresholded, Mat imgThresholdedW, Point2D &koordinat, int &ukuran)
{

    static int ada1, tiada1;
    int ballsize, banyakdeteksi;
    bool adaputih, adahijau;

    adaputih = false;
    adahijau = false;
    ukuran = -1;
    banyakdeteksi = 0;

    //Ubah winStride dan scale untuk memvariasikan kecepatan vs accuracy trade-off
    //winstride dan scale semakin besar semakin cepat, tapi akurasi berkurang
    vector<Rect> ball;

    Size padding(Size(8, 8));
    Size winStride(Size(8, 8));
    double scale = 1.1;

    hog.detectMultiScale(frame, ball, hitThreshold, winStride, padding, scale);

    //Expectation should have only detect 1 ball or none
        fprintf(stderr, "Banyak deteksi : %d\n", ball.size());

    if(ball.size() > 0){

        for(size_t i = 0; i < ball.size(); i++) //maybe try to add ball size limit relative to camera resolution based on ball area in pixel
        {
            for(int k = -20; k <=20; k+=2 ){
                for(int j = -20; j <=20; j+=2 ){
                    if (imgThresholdedW.at<uchar>((ball[i].y + ball[i].height*0.5 + k),(ball[i].x + ball[i].width*0.5 + j)) == 255){
                        adaputih = true;
                    }
                    else {
                        adaputih = false;
                    }                
                }
            }

            if (imgThresholded.at<uchar>(ball[i].y + ball[i].height*0.5, ball[i].x + ball[i].width*0.5) == 0){
                adahijau = false;
            }
            else
            {
                adahijau = true;
            }

            if(adaputih == true && adahijau == false)
            {
//                koordinat.X += ball[i].x + ball[i].width*0.5;
//                koordinat.Y += ball[i].y + ball[i].height*0.5;
               koordinat.X = ball[i].x + ball[i].width*0.5;
               koordinat.Y = ball[i].y + ball[i].height*0.5;
                // Center.x = koordinat.X;
                // Center.y = koordinat.Y;
                ballsize += ball[i].height*0.5;
                // // circle center
                // circle( frame, Center, 3, Scalar(0,255,0), -1, 8, 0 );
                // // circle outline
                // circle( frame, Center, ballsize, Scalar(0,0,255), 3, 8, 0 );
                banyakdeteksi++;
                break;
            }
        }

        // if(banyakdeteksi > 0)
        // {
        //     koordinat.X /= banyakdeteksi;
        //     koordinat.Y /= banyakdeteksi;
        //     ballsize /= banyakdeteksi;
        // }
        // else
        // {
        //     koordinat.X = -1;
        //     koordinat.Y = -1;
        //     ballsize = 0;
        // }
        // ukuran = ballsize;                        

    }
    else
    {
        koordinat.X = -1;
        koordinat.Y = -1;
    }


    // imshow("Result", frame);        

//    koordinat.X = koordinat.X; //* size.width / size1.width;
//    koordinat.Y = koordinat.Y; //* size.width / size1.width;

    //koordinat.Y += 50;
    if(koordinat.Y > size.height)
    {
        koordinat.Y = size.height;
    }
    if(koordinat.X > size.width)
    {
        koordinat.X = size.width;
    }
/*
    selisih.X = abs(koordinat.X - koordinatlama.X);
    selisih.Y = abs(koordinat.Y - koordinatlama.Y);

    if(koordinat.X > -1 && koordinat.Y > -1){
        adabola = true;
        if(selisih.X < size.width/2 || selisih.Y < size.height/2){
            for(int i = 4; i >= 0; i--){
                deteksi[i] = deteksi[i-1];
            }
            if(ada1 < 5){
                ada1++;
            }
            deteksi[0] = koordinat;
        }        

    } 
    else{
        adabola = false;
        tiada1++;
    }

    if(ada1 > 0){
        koordinat.X = 0;
        koordinat.Y = 0;

        for(int i = 0; i < ada1; i++){
            koordinat.X += deteksi[i].X;
            koordinat.Y += deteksi[i].Y;
        }
        koordinat.X /= ada1;            
        koordinat.Y /= ada1;                
    }


    koordinatlama = koordinat; 
    
    if(tiada1 == 5){
        for(int i = 0; i < 5; i++){
            deteksi[i].X = 0;
            deteksi[i].Y = 0;
        }
        koordinatsimpan = koordinat;
        tiada1 = 0;
        ada1 = 0;
    }
*/
}


void ImgProcess::houghcircling(Point2D &koordinat, Mat imgThresholded, Mat imgThresholdedW, int &radius)
{
    vector<Vec3f> circles;

    HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, dp, mindist,
                 cannyhough, centerthresh, minrad, maxrad // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );

    bool putih, udah, hijauy;

    int csize;
    csize = 0;
    udah = false;

// Ini lagi dicoba ngelimit jumlah circlesnya
    for( size_t i = 0; i < circles.size(); i++ )
    {
        if(i > 30){
            break;
        }
     
        Point center1(cvRound(circles[i][0]), cvRound(circles[i][1]));
//            koordinat.X += center.x;
//            koordinat.Y += center.y;

        Size sizeputih = imgThresholdedW.size();

        if (imgThresholded.at<uchar>(center1.y, center1.x) == 0)
        {
            hijauy = true;
        }
        else
        {
            hijauy = false;
        }

        for(int k = -60; k <=20; k+= 2)
        {
            if(k < 0)
            {
                k = 0;
            }
            if(k > sizeputih.height)
            {
                k = sizeputih.height;
            }
            for(int j = -50; j <=50; j+= 2)
            {
                if(j < 0)
                {
                    j = 0;
                }
                if(j > sizeputih.width)
                {
                    j = sizeputih.width;
                }
                if (imgThresholdedW.at<uchar>((center1.y * size1.width / size.width + k),(center1.x * size1.width / size.width + j)) == 255)
                {
                    putih = true;
                    break; 
                }
                else 
                {
                    putih = false;
//                        csize --;
                    //
                }                
            }

            if(putih == true)
            {
                break;
            }
        }

        int radiusgawang = cvRound(circles[i][2]) * size1.width / size.width;
        int adagawang = 0;
        bool inigawang = false;

        if(putih == true && hijauy == true)
        {
            for(int zz = 0; zz < size1.height; zz += radiusgawang/2)
            {
                if (imgThresholdedW.at<uchar>((center1.y * size1.width / size.width - zz), center1.x) == 255)
                {
                    adagawang++;

                    if(center1.y * size1.width / size.width - zz < 0)
                    {
                        break;
                    }
                }
            }

            if(adagawang > 3)
            {
                inigawang = true;
            }
            else
            {
                adagawang = 0;
                inigawang = false;
            }
        }

        if(putih == true && hijauy == true /*&& inigawang == false*/){

            udah = true;

//Uncomment sekitar sini biar keliatan semua titik
            // radius = cvRound(circles[i][2]);

            // center.x = koordinat.X;
            // center.y = koordinat.Y;

            // circle center
            // circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // // circle outline
            // circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );

            if(center1.x > -1 && center1.y > -1)
            {
                koordinat.X += (int)center1.x;
                koordinat.Y += (int)center1.y;                    
                csize ++;

                radius = cvRound(circles[i][2]) * size1.width / size.width;
//                        radii = radius;
            }
        }    


        if(i == (circles.size() - 1) && udah == true && csize != 0)
        {

            koordinat.X /= (int)csize;
            koordinat.Y /= (int)csize;

            if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < 0)
            {
                koordinat.Y -= 20;
                // if(koordinat.X > 160){
                //     koordinat.X += 7;
                // } else koordinat.X -= 7;
            } else
            if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < 0)
            {
                koordinat.Y += 40;
                // if(koordinat.X > 160){
                //     koordinat.X += 7;
                // } else koordinat.X -= 7;
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -3)
            {
                koordinat.Y += 50;
                // if(koordinat.X > 160){
                //     koordinat.X += 10;
                // } else koordinat.X -= 10;
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -6)
            {
                koordinat.Y += 60;
                // if(koordinat.X > 160){
                //     koordinat.X += 15;
                // } else koordinat.X -= 15;
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -9)
            {
                koordinat.Y += 70;
                // if(koordinat.X > 160){
                //     koordinat.X += 20;
                // } else koordinat.X -= 20;
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -12)
            {
                koordinat.Y += 80;
                // if(koordinat.X > 160){
                //     koordinat.X += 25;
                // } else koordinat.X -= 25;
            
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -15)
            {
                koordinat.Y += 100;
                // if(koordinat.X > 160){
                //     koordinat.X += 30;
                // } else koordinat.X -= 30;
            } else if(MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT) < -18)
            {
                koordinat.Y += 120;
                // if(koordinat.X > 160){
                //     koordinat.X += 35;
                // } else koordinat.X -= 35;
            }

            if(koordinat.Y > size.height)
            {
                koordinat.Y = size.height;
            }

            if(koordinat.Y < 0)
            {
                koordinat.Y = 0;
            }

            if(koordinat.X > size.width)
            {
                koordinat.X = size.width;
            }

            if(koordinat.X < 0)
            {
                koordinat.X = 0;
            }
            // if(koordinat.X > 320){
            //     koordinat.X = 320;
            // } else if(koordinat.X < -1){
            //     koordinat.X = 0;
            // }
            // koordinat.Y += 15;              
        }

    }


    selisih.X = abs(koordinat.X - koordinatlama.X);
    selisih.Y = abs(koordinat.Y - koordinatlama.Y);

    if(koordinat.X > -1 && koordinat.Y > -1)
    {
        adabola = true;
        if(selisih.X < 75 || selisih.Y < 75)
        {
            for(int i = 4; i >= 0; i--)
            {
                deteksi[i] = deteksi[i-1];
            }
            if(ada < 5)
            {
                ada++;
            }
            deteksi[0] = koordinat;
        }        

    } 
    else
    {
        // adabola = false;
        tiada++;
    }

    if(ada > 0)
    {
        koordinat.X = -1;
        koordinat.Y = -1;

        for(int i = 0; i < ada; i++)
        {
            koordinat.X += deteksi[i].X;
            koordinat.Y += deteksi[i].Y;
        }
        koordinat.X /= ada;            
        koordinat.Y /= ada;                
    }
    // else
    // {
    //     koordinat.X = -1;
    //     koordinat.Y = -1;
    // }


    koordinatlama = koordinat; 
    
    if(tiada > 3)
    {
        for(int i = 0; i < 5; i++)
        {
            deteksi[i].X = 0;
            deteksi[i].Y = 0;
        }
        if(koordinat.X != -1 && koordinat.Y != -1)
        {
            koordinatsimpan = koordinat;
        }
        adabola = false;
        tiada = 0;
        ada = 0;
    }
   

     // printf("\n======== circles size : %d", (int)circles.size());
     // printf("\n--------------------------------\n\n");
     // printf("\nKoordinat X  : %d ", (int)koordinat.X);
     // printf("\nKoordinat Y  : %d ", (int)koordinat.Y);
     // // printf("Selisih X    : %d \n", (int)selisih.X);
     // // printf("Selisih Y    : %d \n", (int)selisih.Y);
     // printf("\nKoordinat SX : %d ", (int)koordinatsimpan.X);
     // printf("\nKoordinat SY : %d ", (int)koordinatsimpan.Y);
     // printf("\nRadius       : %d ", (int)radii);
     // printf("\n---------------------------------\n");
    
}



void ImgProcess::thresholding(Mat src, Mat &imgThresholded, Mat &imgThresholdedW)
{
    Mat imgHSV;

    cvtColor(src, imgHSV, CV_RGB2HSV); //Convert the captured frame from RGB to HSV
//    cvtColor(src, src, CV_RGB2BGR); //Convert the captured frame from RGB to HSV

    int imgWidth = size.width;
    int imgHeight = size.height;

    imgThresholded.create(imgWidth, imgHeight, CV_8UC1);

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    inRange(imgHSV, Scalar(wLowH, wLowS, wLowV), Scalar(wHighH, wHighS, wHighV), imgThresholdedW); //Threshold the image


    //morphological closing (fill small holes in the foreground)                      
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(dilategreen, dilategreen)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(erodegreen, erodegreen)) );
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(erodegreen, erodegreen)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(dilategreen, dilategreen)) ); 

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholdedW, imgThresholdedW, getStructuringElement(MORPH_ELLIPSE, Size(erodewhite, erodewhite)) );
    dilate( imgThresholdedW, imgThresholdedW, getStructuringElement(MORPH_ELLIPSE, Size(dilatewhite, dilatewhite)) ); 
    //morphological closing (fill small holes in the foreground) 
    dilate( imgThresholdedW, imgThresholdedW, getStructuringElement(MORPH_ELLIPSE, Size(dilatewhite, dilatewhite)) ); 
    erode(imgThresholdedW, imgThresholdedW, getStructuringElement(MORPH_ELLIPSE, Size(erodewhite, erodewhite)) );

    resize(imgThresholded, imgThresholded, size);
}

void ImgProcess::SaveParams()
{
    // if(saving == 1 && ceksave1 != 1){
    //     SaveColorSettings(ini, "WHITE", 
    //                       wLowH, wHighH, wLowS, wHighS, wLowV, wHighV);
    //     LinuxCamera::GetInstance()->SaveINISettings(ini);
    //     printf("\n\nParams saved!");
    //     ceksave1 = 1;
    //     ceksave0 = 0;
    // } else if(saving == 0 && ceksave0 != 1){
    //     ceksave1 = 0;
    //     ceksave0 = 1;
    // }

    // if(savingc == 1 && ceksavec1 != 1){
    //     SaveColorSettings(ini, "GREEN", 
    //                       iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);

    //     printf("\n\nParams saved!");
    //     ceksavec1 = 1;
    //     ceksavec0 = 0;
    // } else if(savingc == 0 && ceksavec0 != 1){
    //     ceksavec1 = 0;
    //     ceksavec0 = 1;
    // }

    // if(loading == 1 && cekload1 != 1){
    //     LoadColorSettings(ini, "WHITE", 
    //                       &wLowH, &wHighH, &wLowS, &wHighS, &wLowV, &wHighV);
    //     LinuxCamera::GetInstance()->LoadINISettings(ini);
    //     cvSetTrackbarPos("LowH", "Kamera", wLowH); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighH", "Kamera", wHighH);
    //     cvSetTrackbarPos("LowS", "Kamera", wLowS); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighS", "Kamera", wHighS);
    //     cvSetTrackbarPos("LowV", "Kamera", wLowV); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighV", "Kamera", wHighV);

    //     printf("\n\nParams loaded!");
    //     cekload1 = 1;
    //     cekload0 = 0;
    // } else if(loading == 0 && cekload0 != 1){
    //     cekload1 = 0;
    //     cekload0 = 1;
    // }

    // if(loadingc == 1 && cekloadc1 != 1){
    //     LoadColorSettings(ini, "GREEN", 
    //                       &iLowH, &iHighH, &iLowS, &iHighS, &iLowV, &iHighV);
    //     cvSetTrackbarPos("LowH", "Control", iLowH); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighH", "Control", iHighH);
    //     cvSetTrackbarPos("LowS", "Control", iLowS); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighS", "Control", iHighS);
    //     cvSetTrackbarPos("LowV", "Control", iLowV); //Hue (0 - 179)
    //     cvSetTrackbarPos("HighV", "Control", iHighV);

    //     printf("\n\nParams loaded!");

    //     cekloadc1 = 1;
    //     cekloadc0 = 0;
    // } else if(loadingc == 0 && cekloadc0 != 1){
    //     cekloadc1 = 0;
    //     cekloadc0 = 1;
    // }    
}