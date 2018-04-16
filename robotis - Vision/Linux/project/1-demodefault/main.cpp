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
#include <pthread.h>

#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"
#include "minIni.h"
#include "udp_receiver.cpp"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

uint16_t gamestate;     // state of the game (STATE_READY, STATE_PLAYING, etc)
uint16_t firsthalf;     // 1 = game in first half, 0 otherwise
uint16_t kickoffteam;   // the next team to kick off (TEAM_BLUE, TEAM_RED)
uint16_t secgamestate;  // Extra state information - (STATE2_NORMAL, STATE2_PENALTYSHOOT, etc)
uint16_t dropinteam;    // team that caused last drop in
uint32_t dropintime;    // number of seconds passed since the last drop in.  -1 before first dropin
uint32_t secsremaining; // estimate of number of seconds remaining in the half
uint32_t secondarytime; // number of seconds shown as secondary time (remaining ready, until free ball, etc)

pthread_mutex_t kompas;
pthread_mutex_t gamecontroller;

//using namespace Robot;
using namespace std;
using namespace cv;


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


void *GController(void *t){

//    char ipaddress [12] = "127.0.0.1";
    GameController GC = GameController(NULL, 3838);
//    GC = GameController(NULL, 3838);

    GC.start();

    int n = 0;

    while(1){

        //RS.serialWriteReady();
        //if(i%100000==0){

        n = GC.receive_broadcast();

        printf("\n n = %d", n);
        if(n < 120){
            pthread_mutex_lock(&gamecontroller);

            gamestate = GC.getGameState();
            secondarytime = GC.getSecondaryTime();
            kickoffteam = GC.getKickOff();
            secgamestate = GC.getSecGameState();
            secsremaining = GC.getRemainingSecs();

            pthread_mutex_unlock(&gamecontroller);                                
        }
    }
    pthread_exit(NULL);
}


int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

//    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    //LinuxCamera::GetInstance()->Initialize(0);
    //LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    //LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    ImgProcess vision = ImgProcess();

    Mat src, imgThresholded, imgThresholdedW;
    Point2D koordinat, ballposlama;
    int bright, saturasi, contras, ga, expo, saving, savingc, loading, loadingc;

    // ga = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_BRIGHTNESS);
    ga = 120;
    // expo = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_BRIGHTNESS);
    expo = 100;
    bright = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_BRIGHTNESS);
    // saturasi = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_SATURATION);
    saturasi = 75;
    contras = Robot::LinuxCamera::GetInstance()->v4l2GetControl(V4L2_CID_CONTRAST);

    namedWindow("Game", CV_WINDOW_AUTOSIZE);
    namedWindow("Kamera", CV_WINDOW_AUTOSIZE);
    namedWindow("Result", CV_WINDOW_AUTOSIZE);
    namedWindow("Thresholded", CV_WINDOW_AUTOSIZE);

    VideoCapture cap;
    cap.open(0);

    cap.set(CAP_PROP_FRAME_WIDTH, Camera::WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, Camera::HEIGHT);

    // if(!cap.isOpened())  // check if we succeeded
    //     return -1;

    // src = Mat(Camera::WIDTH, Camera::HEIGHT, CV_8UC3);
    src = Mat(Camera::HEIGHT, Camera::WIDTH, CV_8UC3);

    char key;

/*
    //pthread untuk kompas dan game controller
    pthread_t threads[2];
    pthread_attr_t attr[2];
    long t1 = 1; long t2 = 2;

    pthread_mutex_init(&kompas, NULL);
    pthread_mutex_init(&gamecontroller, NULL);
    pthread_attr_init(&attr[0]);
    pthread_attr_init(&attr[1]);

    pthread_create(&threads[0], &attr[0], ambildatakompas, (void *) t1);
    pthread_create(&threads[1], &attr[1], GController, (void *) t2);
*/

    int i = 0;

    int baud = 9600;

    SerialHandler::GetInstance()->Initialize(baud);

    printf("hayo\n");

//    usleep(1000000);

    HOGDescriptor hog; // Use standard parameters here
        
    String hogfile;
    double hitThreshold;

    hogfile = "../../../Data/cvHOGClassifier.yaml";

    hog.winSize = Size(64, 64); // Default training images size as used in paper

//    hog.setSVMDetector(descriptorVector);
    hog.load(hogfile);
    
    hitThreshold = 1.670206;

//    int kVal[3];
//    int sVal[2];
    int aVal;

    int _ball_found = 0;
    while(1)
    {
        char mode = 'x';

        cap >> src;

/*
        //melakukan thresholding hijau dan putih pada src
        vision.thresholding(src, imgThresholded, imgThresholdedW);


        int radius;
        Point center;

        radius = 0;

        koordinat.X = -1;
        koordinat.Y = -1;

        center.x = -1;
        center.y = -1;

        
        vision.featuredetect(src, imgThresholded, imgThresholdedW, koordinat, radius);

        center.x = koordinat.X;
        center.y = koordinat.Y;

        if(koordinat.X != -1 && koordinat.Y != -1 && radius > 0)
        {
            // melingkari hasil deteksi bole        
            // circle center
            circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        fprintf(stderr, "Koordinat X : %d\n", koordinat.X);

        // printf("\nKoordinat X : %d", (int)koordinat.X);
        // printf("\nKoordinat Y : %d", (int)koordinat.Y);        
*/


        vector<Rect> ball;

        Size padding(Size(8, 8));
        Size winStride(Size(8, 8));
        double scale = 1.2;
        Point center;
        hog.detectMultiScale(src, ball, hitThreshold, winStride, padding, scale);

        //cvtColor(testImage, testImage, CV_BGR2GRAY); // Work on grayscale images as trained

//        detectTest(hog, hitThreshold, testImage);

        fprintf(stderr, "Banyak deteksi : %d\n", ball.size());

        if(ball.size() > 0){
            for(size_t i = 0; i < ball.size(); i++) //maybe try to add ball size limit relative to camera resolution based on ball area in pixel
            {
                int ballsize;

                center.x = ball[i].x + ball[i].width*0.5;
                center.y = ball[i].y + ball[i].height*0.5;
                ballsize = ball[i].height*0.5;

                circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( src, center, ballsize, Scalar(0,0,255), 3, 8, 0 );

            }
        }

        imshow("Result", src);
//        imshow("Thresholded", imgThresholded);
//        imshow("TWhite", imgThresholdedW);


        key = cvWaitKey(10);
        if (char(key) == 'a'){
            char filename[80];
            sprintf(filename,"data/test_%d.png",i);
            imwrite(filename, src);
            i++;
        }

        if(char(key) == 'x')
        {
            break;
        }
        
//         if(key == 'j')
//         { 
//             printf("eh masuk");
//             char bufferWrite[] = {"test1"};
//             kirimChar(bufferWrite, port);
// //            ambilChar();
// //            printf("%d",nser);
//         }

//         if(key == 'f')
//         { 
//             printf("eh masuk");
//             char bufferWrite[] = {"test2"};
//             kirimChar(bufferWrite, port);
// //            ambilChar();
// //            printf("%d",nser);
//         }

        _ball_found = tracker.SearchAndTracking(koordinat);

        // if(Action::GetInstance()->IsRunning() == 0 &&
        //         StatusCheck::m_soccer_sub_mode == SOCCER_PLAY)
        // {
        //     Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        //     Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
            
        if(_ball_found != 1){
           SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
           SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
           SerialHandler::GetInstance()->WalkingStart();
        }

        if(_ball_found == 1)
        {
            follower.Process(tracker.ball_position);

            if(follower.KickBall != 0)
            {
//                    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                    Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                if(follower.KickBall == -1)
                {
                    aVal = 12;
                    SerialHandler::GetInstance()->SendAction(aVal);
//                        Action::GetInstance()->Start(12);   // RIGHT KICK
                    fprintf(stderr, "RightKick! \n");
                }
                else if(follower.KickBall == 1)
                {
                    aVal = 13;
                    SerialHandler::GetInstance()->SendAction(aVal);
//                        Action::GetInstance()->Start(13);   // LEFT KICK
                    fprintf(stderr, "LeftKick! \n");
                }
            }
        }
        else if(_ball_found == -1)
        {
            SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = 10.0;
        }
        else
        {
            SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = -1.0;
            SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
        }

        SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = -20.0;
        SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = 20.0;

        SerialHandler::GetInstance()->SendKinematics();

        // }

//         bool cekready = false;
//         int lastballfound, hitungmundur;
//         if(StatusCheck::m_cur_mode == SOCCER || StatusCheck::m_cur_mode == VISION || cekready == true)
//         {
//             //tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
//             Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//             _ball_found = tracker.SearchAndTracking(koordinat, abisnendang, head_pan_anglelama, vision.size);

//                 // Head::GetInstance()->MoveByAngle(0, -25);
//             // printf("\nDapet bola? %d", (int)_ball_found);
            
//             if(lastballfound != _ball_found){
//                 lastballfound = _ball_found;
//             }

//             if(lastballfound == 1){
//                 hitungmundur++;
//             } else hitungmundur = 0;

//             if(head_pan_anglelama == Head::GetInstance()->GetPanAngle() && head_tilt_anglelama == Head::GetInstance()->GetTiltAngle())
//             {
//                 hitungleher++;
//             }
//             else
//             {
//                 hitungleher = 0;
//             }

//             if(hitungleher > 30)
//             {
//                 Head::GetInstance()->MoveByAngle(0, -25);
//                 hitungleher = 0;
//             }

//         }




//         int statuslama, hitungawalmaju;
//         bool pernahsoccer, dariulang, majudulu;


//         if(StatusCheck::m_cur_mode == SOCCER && statuslama != SOCCER){
//             dariulang = true;
//         } else dariulang = false;

//         if(StatusCheck::m_cur_mode == VISION && statuslama != VISION){
//             dariulang = true;
//         } else dariulang = false;

//         if(dariulang == true){
//             majudulu = true;
//             hitungada = 0;
//         } else majudulu = false;

//         if(statuslama != StatusCheck::m_cur_mode)
//         {
//             statuslama = StatusCheck::m_cur_mode;
//         }


//         if(StatusCheck::m_is_started == 0)
//             continue;

// //////////////////////////////////////// Select Mode ///////////////////////
//         switch(StatusCheck::m_cur_mode)
//         {
//         case READY:
//             if(awal == false){
//                 awal = true;
//             }
//             break;
//         case SOCCER:
//             if(Action::GetInstance()->IsRunning() == 0 && abisjatoh == true &&
//                     StatusCheck::m_soccer_sub_mode == SOCCER_PLAY && statemulai == true)
//             {
//                 hitungjatoh++;
//                 if(hitungjatoh == 1){
//                     Head::GetInstance()->MoveByAngle(0, -25);
//                 }
//                 if(hitungjatoh > 5){
//                     abisjatoh = false;
//                     hitungjatoh = 0;
//                     Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                     Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//                 }
//             } else
//             if(Action::GetInstance()->IsRunning() == 0 &&
//                     StatusCheck::m_soccer_sub_mode == SOCCER_PLAY && statemulai == true)
//             {
//                 pilihmode = 1;
//                 pernahsoccer = true;

//                 if(awal == true){
//                     Head::GetInstance()->Initialize();
//                     awal = false;
//                 }

//                 if(majudulu == true)
//                 {
//                     if(_ball_found == 1)
//                     {
//                         hitungada++;
//                     }

//                     if(hitungada > 30)
//                     {
//                         hitungada = 0;
//                         dariulang = false;
//                         _ball_found = 1;
//                     }
//                     else
//                     {
//                         hitungada = 0;
//                         _ball_found = -1;
//                     }
//                 }

//                 Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

//                 if(Walking::GetInstance()->IsRunning() == false && _ball_found != 1){
//             		Walking::GetInstance()->X_MOVE_AMPLITUDE = -4.0;
//             		Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//             		Walking::GetInstance()->Start();
//             	}

//                 if(abisnendang == true && _ball_found == -1)
//                 {
//                     Walking::GetInstance()->X_MOVE_AMPLITUDE = -2.0;
//                     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                     Walking::GetInstance()->Start();                          
//                 }

//                 if(majudulu == true && _ball_found == -1){
//                     printf("\n############Maju dulu dong#############");
//                     Walking::GetInstance()->X_MOVE_AMPLITUDE = 5.0;
//                     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                     Walking::GetInstance()->Start();      
//                     hitungawalmaju++;              
//                 } else

//                 if(_ball_found == 1)
//                 {
//                     majudulu = false;
//                     // if(dariulang == true)
//                     // {
//                     //     hitungulang++;
//                     //     if(hitungulang > 5)
//                     //     {
//                     //         dariulang = false;
//                     //     }
//                     // }

//                     hitungawalmaju = 0; hitungmuter = 0; hitungmundur = 0;

//                     follower.Process(tracker.ball_position, koordinat, vision.adabola, datakompas, arahgawang, pilihmode);
                                      
//                     if(follower.KickBall != 0)
//                     {
//                         head_pan_anglelama = Head::GetInstance()->GetPanAngle();
//                         head_tilt_anglelama =  Head::GetInstance()->GetTiltAngle();

//                          Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                          Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

//                          usleep(8000);
//                         if(follower.KickBall == -2)
//                         {
//                             Action::GetInstance()->Start(96);   // LEFT MUTER
//                             fprintf(stderr, "MuterKiri! \n");   
//                         }
//                         else if(follower.KickBall == 2){
//                             Action::GetInstance()->Start(95);   // RIGHT MUTER
//                             fprintf(stderr, "MuterKanan! \n");                               
//                         }
//                         if(follower.KickBall == -1)
//                         {
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             Action::GetInstance()->Start(110);   // RIGHT KICK
//                             fprintf(stderr, "RightKick! \n");
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             usleep(50000);
//                             abisnendang = true;
//                         }
//                         else if(follower.KickBall == 1)
//                         {
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             Action::GetInstance()->Start(111);   // LEFT KICK
//                             fprintf(stderr, "LeftKick! \n");
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             usleep(50000);
//                             abisnendang = true;
//                         }
//                     }
//                     head_pan_anglelama = Head::GetInstance()->GetPanAngle();
//                     head_tilt_anglelama =  Head::GetInstance()->GetTiltAngle();
//                     ballposlama = tracker.ball_position;

//                 }
//                 else if(_ball_found == -1)
//                 {

//                     if(dariulang == true){
//                         hitungulang = 0;
//                         Walking::GetInstance()->X_MOVE_AMPLITUDE = 12.0;
//                         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                                                
//                     } else
//                     if(hitungmundur > 10 && hitungmundur < 75){
//                         Walking::GetInstance()->X_MOVE_AMPLITUDE = -7.0;
//                         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                                                
//                     }            
//                     else
//                     {
//                         // hitungmundur = 0;
//                         hitungmuter++;

//                         // if(hitungmaju > 20)
//                         // {
//                         //     Walking::GetInstance()->X_MOVE_AMPLITUDE = 3.0;
//                         //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         //     hitungjalan++;
//                         //     if(hitungjalan > 5){
//                         //         Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                         //         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         //         hitungmaju = 0;
//                         //         hitungmuter = 0;
//                         //     }
//                         // }else 

//                         if(hitungmuter > 30)
//                         {
//                             if(ballposlama.X > 0)
//                             {
//                                 Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                                 Walking::GetInstance()->A_MOVE_AMPLITUDE = -9.0;                                
//                             }
//                             else
//                             {
//                                 Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                                 Walking::GetInstance()->A_MOVE_AMPLITUDE = 9.0;                                
//                             }
//                             hitungberputar++;
//                             if(hitungberputar > 40){
//                                 hitungmuter = 0;
//                                 Walking::GetInstance()->X_MOVE_AMPLITUDE = -2.0;
//                                 Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                                 // hitungberputar = 0;
//                                 // hitungmaju++;
//                                 // printf("\n==== Sudah Muter yeeey ===");                            
//                             }
//                         }
//                         else
//                         {
//                             Walking::GetInstance()->X_MOVE_AMPLITUDE = 0.0;
//                             Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         }

//                         // printf("\nUdah muter        : %d", tracker.udahmuter);
//                         // printf("\nHitung Muter      : %d", hitungmuter);
//                         // printf("\nHitung Berputar   : %d", hitungberputar);
//                         // printf("\nHitung Maju       : %d", hitungmaju);                        
//                     }
//                 }
//                 else
//                 {
//             		Walking::GetInstance()->X_MOVE_AMPLITUDE = -3.0;
//             		Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                 }

//             }


//             if(Walking::GetInstance()->IsRunning() == false && statemulai == true && abisjatoh == false){
//                 Walking::GetInstance()->X_MOVE_AMPLITUDE = -4.0;
//                 Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                 Walking::GetInstance()->Start();
//             }                
//             else if(statemulai == false)
//             {
//                 if(Walking::GetInstance()->IsRunning() == true)
//                 Walking::GetInstance()->Stop();
//             }

//             head_pan_anglelama = Head::GetInstance()->GetPanAngle();
//             head_tilt_anglelama =  Head::GetInstance()->GetTiltAngle();

//             break;

//         case MOTION:
//             // if(LinuxActionScript::m_is_running == 0)
//             //     LinuxActionScript::ScriptStart(SCRIPT_FILE_PATH);
//             cm730.WriteByte(19, MX28::P_TORQUE_ENABLE, 0, NULL);
//             cm730.WriteByte(20, MX28::P_TORQUE_ENABLE, 0, NULL);
                
//             break;


//         case VISION:
//             if(Action::GetInstance()->IsRunning() == 0 && abisjatoh == true &&
//                     StatusCheck::m_soccer_sub_mode == SOCCER_PLAY && statemulai == true)
//             {
//                 hitungjatoh++;
//                 if(hitungjatoh == 1){
//                     Head::GetInstance()->MoveByAngle(0, -25);
//                 }
//                 if(hitungjatoh > 5){
//                     abisjatoh = false;
//                     hitungjatoh = 0;
//                     Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                     Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
//                 }
//             } else
//             if(Action::GetInstance()->IsRunning() == 0 &&
//                     StatusCheck::m_soccer_sub_mode == SOCCER_PLAY && statemulai == true)
//             {
//                 pilihmode = 1;
 
//                 pernahsoccer = true;

//                 if(awal == true){
//                     Head::GetInstance()->Initialize();
//                     while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                     awal = false;
//                 }

//                 // if(hitungada != -1)
//                 // {
//                 //     dariulang = false;
//                 //     majudulu = false;
//                 // }

//                 // if(majudulu == true)
//                 // {
//                 if(_ball_found == 1 && hitungada != -1)
//                 {
//                     hitungada++;
//                     printf("\nHitungada : %d", hitungada);
//                 }

//                 if(hitungada != -1)
//                 {
//                     if(hitungada > 3)
//                     {
//                         hitungada = -1;
//                         dariulang = false;
//                         majudulu = false;
//                         _ball_found = 1;
//                     }
//                     else
//                     {
//                         dariulang = true;
//                         majudulu = true;
//                         if(_ball_found == -1)
//                         {
//                             hitungada = 0;                 
//                         }
//                         _ball_found = -1;
//                     }                    
//                 }
//                 // }

//                 Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                 Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

//                 if(Walking::GetInstance()->IsRunning() == false && _ball_found != 1){
//                     Walking::GetInstance()->X_MOVE_AMPLITUDE = -4.0;
//                     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                     Walking::GetInstance()->Start();
//                 }

//                 if(abisnendang == true && _ball_found == -1)
//                 {
//                     Walking::GetInstance()->X_MOVE_AMPLITUDE = -2.0;
//                     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                     Walking::GetInstance()->Start();                          
//                 } else

//                 // if(majudulu == true && _ball_found == -1){
//                 //     printf("\n############Maju dulu dong#############");
//                 //     Walking::GetInstance()->X_MOVE_AMPLITUDE = 5.0;
//                 //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                 //     Walking::GetInstance()->Start();      
//                 //     hitungawalmaju++;              
//                 // } else

//                 if(_ball_found == 1)
//                 {
//                     majudulu = false;
//                     dariulang = false;
//                     // if(dariulang == true)
//                     // {
//                     //     hitungulang++;
//                     //     if(hitungulang > 5)
//                     //     {
//                     //         dariulang = false;
//                     //     }
//                     // }

//                     hitungawalmaju = 0; hitungmuter = 0; hitungmundur = 0;

//                     follower.Process(tracker.ball_position, koordinat, vision.adabola, datakompas, arahgawang, pilihmode);
                                      
//                     if(follower.KickBall != 0)
//                     {
//                         head_pan_anglelama = Head::GetInstance()->GetPanAngle();
//                         head_tilt_anglelama =  Head::GetInstance()->GetTiltAngle();

//                          Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
//                          Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

//                          usleep(8000);
//                         if(follower.KickBall == -2)
//                         {
//                             Action::GetInstance()->Start(96);   // LEFT MUTER
//                             fprintf(stderr, "MuterKiri! \n");   
//                         }
//                         else if(follower.KickBall == 2){
//                             Action::GetInstance()->Start(95);   // RIGHT MUTER
//                             fprintf(stderr, "MuterKanan! \n");                               
//                         }
//                         if(follower.KickBall == -1)
//                         {
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             Action::GetInstance()->Start(110);   // RIGHT KICK
//                             fprintf(stderr, "RightKick! \n");
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             usleep(50000);
//                             abisnendang = true;
//                         }
//                         else if(follower.KickBall == 1)
//                         {
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             Action::GetInstance()->Start(111);   // LEFT KICK
//                             fprintf(stderr, "LeftKick! \n");
//                             while(Action::GetInstance()->IsRunning() == 1) usleep(8000);
//                             usleep(50000);
//                             abisnendang = true;
//                         }
//                     }
//                 }
//                 else if(_ball_found == -1)
//                 {
//                     if(dariulang == true){
//                         hitungulang = 0;
//                         Head::GetInstance()->MoveByAngle(0, 7);
//                         Walking::GetInstance()->X_MOVE_AMPLITUDE = 12.0;
//                         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                                                
//                         Walking::GetInstance()->Start();
//                     } else
//                     if(hitungmundur > 10 && hitungmundur < 50){
//                         Walking::GetInstance()->X_MOVE_AMPLITUDE = -7.0;
//                         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                                                
//                         Walking::GetInstance()->Start();
//                     }            
//                     else
//                     {
//                         Walking::GetInstance()->X_MOVE_AMPLITUDE = -2.0;
//                         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         Walking::GetInstance()->Start();

//                         // hitungmundur = 0;
//                         // hitungmuter++;

//                         // if(hitungmaju > 20)
//                         // {
//                         //     Walking::GetInstance()->X_MOVE_AMPLITUDE = 3.0;
//                         //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         //     hitungjalan++;
//                         //     if(hitungjalan > 5){
//                         //         Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                         //         Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         //         hitungmaju = 0;
//                         //         hitungmuter = 0;
//                         //     }
//                         // }else 

//                         // if(hitungmuter > 40)
//                         // {
//                         //     Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                         //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 12.0;
//                         //     hitungberputar++;
//                         //     if(hitungberputar > 20){
//                         //         hitungmuter = 0;
//                                 // hitungberputar = 0;
//                                 // hitungmaju++;
//                                 // printf("\n==== Sudah Muter yeeey ===");                            
//                         //     }
//                         // }
//                         // else
//                         // {
//                         //     Walking::GetInstance()->X_MOVE_AMPLITUDE = -5.0;
//                         //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;                        
//                         // }

//                         // printf("\nUdah muter        : %d", tracker.udahmuter);
//                         // printf("\nHitung Muter      : %d", hitungmuter);
//                         // printf("\nHitung Berputar   : %d", hitungberputar);
//                         // printf("\nHitung Maju       : %d", hitungmaju);                        
//                     }
//                 }
//                 // else
//                 // {
//                 //     Walking::GetInstance()->X_MOVE_AMPLITUDE = -3.0;
//                 //     Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                 // }

//             }


//             if(Walking::GetInstance()->IsRunning() == false && statemulai == true && abisjatoh == false){
//                 Walking::GetInstance()->X_MOVE_AMPLITUDE = -4.0;
//                 Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0;
//                 Walking::GetInstance()->Start();
//             }                
//             else if(statemulai == false)
//             {
//                 if(Walking::GetInstance()->IsRunning() == true)
//                 Walking::GetInstance()->Stop();
//             }

//             head_pan_anglelama = Head::GetInstance()->GetPanAngle();
//             head_tilt_anglelama =  Head::GetInstance()->GetTiltAngle();

//             break;
//         }

        // resize(imgThresholded, imgThresholded, size1);

        // resize(imgThresholdedW, imgThresholdedW, size1);
    
    }

/*
    pthread_attr_destroy(&attr[0]);
    pthread_attr_destroy(&attr[1]);
    pthread_mutex_destroy(&kompas);
    pthread_mutex_destroy(&gamecontroller);
    pthread_exit(NULL);
*/
    return 0;
}