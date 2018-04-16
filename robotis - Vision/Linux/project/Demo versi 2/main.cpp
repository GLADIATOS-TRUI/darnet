/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
//#include <pthread.h>


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

//using namespace Robot;
using namespace std;
using namespace cv;

LinuxCM730 linux_cm730(U2D_DEV_NAME0);		//inisialisasi linux cm730
CM730 cm730(&linux_cm730);					//

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);


    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini


//    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

//    ColorFinder* ball_finder = new ColorFinder();
//    ball_finder->LoadINISettings(ini);
//    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

//    ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
//    red_finder->LoadINISettings(ini, "RED");
//    httpd::red_finder = red_finder;

//    ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
//    yellow_finder->LoadINISettings(ini, "YELLOW");
//    httpd::yellow_finder = yellow_finder;

//    ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
//    blue_finder->LoadINISettings(ini, "BLUE");
//    httpd::blue_finder = blue_finder;

//    httpd::ini = ini;


    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    Action::GetInstance()->Start(15);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

//-----------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------- OpenCV dimulai ---------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

    Mat src;
    Mat imgHSV, imgThresholded, imgThresholdedC;

//    CameraSettings* kamera = new CameraSettings();


//    double dbright, dcontras, dsaturasi, dga, dexpo;
    int bright, contras, saturasi, ga, expo;

    ga = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_GAIN);
    expo = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_EXPOSURE_ABSOLUTE);
    bright = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_BRIGHTNESS);
    contras = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_CONTRAST);
    int iLowH = 42;
    int iHighH = 79;

    int iLowS = 73;
    int iHighS = 255;

    int iLowV = 30;
    int iHighV = 255;

    int minrad = 50;
    int maxrad = 331;

    int ilowC = 100;
    int maxlowC = 100;
    int ratio = 3;

    int centerthresh = 20;

    int cannyhough = 200;

    namedWindow("Control", CV_WINDOW_AUTOSIZE);
    namedWindow("Kamera", CV_WINDOW_AUTOSIZE);
    namedWindow("Result", CV_WINDOW_AUTOSIZE);
    namedWindow("Thresholded", CV_WINDOW_AUTOSIZE);


    cvCreateTrackbar("Brightness", "Kamera", &bright, 1000);
    cvCreateTrackbar("Contrast", "Kamera", &contras, 1000); //Saturation (0 - 256)
//    cvCreateTrackbar("Saturation", "Kamera", &saturasi, 1000);
    cvCreateTrackbar("Gain", "Kamera", &ga, 500);
    cvCreateTrackbar("Exposure", "Kamera", &expo, 10000); //Value (0 - 256)

    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);
    
    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);
    
    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
 
    cvCreateTrackbar("MinRad", "Control", &minrad, 1000); //Value (0 - 255)
    cvCreateTrackbar("MaxRad", "Control", &maxrad, 2000);

    cvCreateTrackbar("Center Thresh", "Control", &centerthresh, 500);

    cvCreateTrackbar("Canny Hough", "Control", &cannyhough, 500);

    Point2D koordinat;
    char key;

    int _ball_found = 0;

        src = Mat(Camera::HEIGHT, Camera::WIDTH, CV_8UC3);

//    VideoCapture cap;
//    cap.open(0);

//    if(!cap.isOpened())  // check if we succeeded
//        return -1;

    while(1)
    {
        StatusCheck::Check(cm730);

//        cap >> src;
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        
//        src.data = rgb_output->m_ImageData;
        src.data = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData;

        int saturasilama, galama, expolama, brightlama, contraslama;
        
//        printf("\n\n================= expo : %d", expo);
//        printf("\n\n================= gain : %d", ga);

        if(saturasilama!=saturasi || galama !=ga || expolama !=expo ||
            brightlama != bright || contraslama != contras){
            
            LinuxCamera::GetInstance()->v4l2SetControl(V4L2_CID_GAIN, ga);
            LinuxCamera::GetInstance()->v4l2SetControl(V4L2_CID_EXPOSURE_ABSOLUTE, expo);
            LinuxCamera::GetInstance()->v4l2SetControl(V4L2_CID_BRIGHTNESS, bright);
            LinuxCamera::GetInstance()->v4l2SetControl(V4L2_CID_CONTRAST, contras);
//            cap.set(CV_CAP_PROP_EXPOSURE, dexpo);
//            cap.set(CV_CAP_PROP_GAIN, dga);
//            cap.set(CV_CAP_PROP_SATURATION, dsaturasi);
//            cap.set(CV_CAP_PROP_BRIGHTNESS, dbright);
//            cap.set(CV_CAP_PROP_CONTRAST, dcontras);
        }

//        saturasilama = saturasi;
        galama = ga;
        expolama = expo;
        brightlama = bright;
        contraslama = contras;

/*
        printf("------------------------------------------------------\n\n");
        printf("GYRO FB  : %d \n", MotionStatus::FB_GYRO);
        printf("GYRO RL : %d \n\n", MotionStatus::RL_GYRO);
        printf("------------------------------------------------------");
*/

//        Point2D ball_pos, red_pos, yellow_pos, blue_pos;



//        flip(src, src, 0);
        cvtColor(src, imgHSV, CV_RGB2HSV); //Convert the captured frame from RGB to HSV
        cvtColor(src, src, CV_RGB2BGR); //Convert the captured frame from RGB to HSV

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
        
        GaussianBlur( imgThresholded, imgThresholded, Size(3,3), 0 , 0);

//        cv::Canny(imgThresholded, imgThresholdedC, ilowC, 100, 3);

        vector<Vec3f> circles;

        HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, 10,
                     cannyhough, centerthresh, minrad, maxrad // change the last two parameters
                                    // (min_radius & max_radius) to detect larger circles
                     );

        koordinat.X = -1;
        koordinat.Y = -1;

        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

//            koordinat.X += center.x;
//            koordinat.Y += center.y;

            koordinat.X = center.x;
            koordinat.Y = center.y;

            int radius = cvRound(circles[i][2]);
            // circle center
            circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }
        
//        if(circles.size()>0){
//            koordinat.X /= circles.size();
//            koordinat.Y /= circles.size();
//        }

        printf("------------------------------------------------------\n\n");
        printf("Koordinat X  : %d \n", (int)koordinat.X);
        printf("Koordinat Y  : %d \n", (int)koordinat.Y);
        printf("------------------------------------------------------");

        imshow("Result", src);
        imshow("Thresholded", imgThresholded);
//        imshow("ThresholdedC", imgThresholdedC);

        key = cvWaitKey(50);
        if (char(key) == 27){
            break;
        }

/*
        if(StatusCheck::m_cur_mode == READY || StatusCheck::m_cur_mode == VISION)
        {
//            ball_pos = ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
//            red_pos = red_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
//            yellow_pos = yellow_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
//            blue_pos = blue_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

            unsigned char r, g, b;
            for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
                r = 0; g = 0; b = 0;
                if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                    r = 255;
                    g = 128;
                    b = 0;
                }
                if(red_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 255;
                        g = 0;
                        b = 0;
                    }
                }
                if(yellow_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 255;
                        g = 255;
                        b = 0;
                    }
                }
                if(blue_finder->m_result->m_ImageData[i] == 1)
                {
                    if(ball_finder->m_result->m_ImageData[i] == 1)
                    {
                        r = 0;
                        g = 255;
                        b = 0;
                    }
                    else
                    {
                        r = 0;
                        g = 0;
                        b = 255;
                    }
                }

                if(r > 0 || g > 0 || b > 0)
                {
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
                    rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
                }
            }

//        }
*/
        if(StatusCheck::m_cur_mode == SOCCER)
        {
            //tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
            _ball_found = tracker.SearchAndTracking(koordinat);

//            for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
//            {

/*
                if(ball_finder->m_result->m_ImageData[i] == 1)
                {
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 255;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 128;
                    rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 0;
                }
*/

//            }
        }

//        streamer->send_image(rgb_output);


        if(StatusCheck::m_is_started == 0)
            continue;

        switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
        case SOCCER:
            if(Action::GetInstance()->IsRunning() == 0 &&
                    StatusCheck::m_soccer_sub_mode == SOCCER_PLAY)
            {


                Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
                
                

                if(Walking::GetInstance()->IsRunning() == false && _ball_found != 1){
            		Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            		Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
            		Walking::GetInstance()->Start();
            	}

                if(_ball_found == 1)
                {
                    follower.Process(tracker.ball_position);

                    if(follower.KickBall != 0)
                    {
                        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                        Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);


                        if(follower.KickBall == -1)
                        {
                            Action::GetInstance()->Start(12);   // RIGHT KICK
                            fprintf(stderr, "RightKick! \n");
                        }
                        else if(follower.KickBall == 1)
                        {
                            Action::GetInstance()->Start(13);   // LEFT KICK
                            fprintf(stderr, "LeftKick! \n");
                        }

                    }
                }
                else if(_ball_found == -1)
                {
            		Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            		Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
                }
                else
                {
            		Walking::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            		Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
                }

            }

            break;

        case MOTION:
            if(LinuxActionScript::m_is_running == 0)
                LinuxActionScript::ScriptStart(SCRIPT_FILE_PATH);
            break;
        case VISION:
            int detected_color = 0;
//            detected_color |= (red_pos.X == -1)? 0 : VisionMode::RED;
//            detected_color |= (yellow_pos.X == -1)? 0 : VisionMode::YELLOW;
//            detected_color |= (blue_pos.X == -1)? 0 : VisionMode::BLUE;

            if(Action::GetInstance()->IsRunning() == 0)
                VisionMode::Play(detected_color);
            break;
        }
    }

    return 0;
}
