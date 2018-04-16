void scanBolaDeket(Mat imgIjo, Mat imgPutih, Point2D &koordinat){
    Size imej = imgIjo.size();
    bool scanIjo, scanDone;
    scanIjo = false; scanDone = false;
    int dapet, dapet2, ceklubang, ceky, x, y, dapetp;
    // int get=0;
    // int getX[3];
    // int getY[3];
    // int xTotal=0;
    // int yTotal=0;
    ceklubang = 0;
    dapet = -1000; dapet2 = 1000; dapetp = 0;

    for(y = imej.height - 160; y >= (imej.height / 2) - 45 ; y-= 10){

        for(x=20 ; x < imej.width - 40; x+=2){
            if(imgIjo.at<uchar>(y,x) == 0){
                dapetp = 0;
                ceklubang = 0;
                for(int m = 0; m <= 10; m++){
                    if(imgIjo.at<uchar>(y, x + m) == 0){ 
                        ceklubang++;
                    }
                    if(imgPutih.at<uchar>(y * 320 / 480, (x + m)  * 320 / 480) == 255){
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
                if(imgIjo.at<uchar>(y,x) == 0){
                    dapetp = 0;
                    ceklubang = 0;
                    for(int m = 0; m <= 10; m++){
                        if(imgIjo.at<uchar>(y,x - m) == 0){ 
                            ceklubang++;
                        }
                        if(imgPutih.at<uchar>(y  * 320 / 480, (x - m) * 320 / 480) == 255){
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

}

void featuredetect(Mat frame, Mat imgThresholdedW, Point2D *koordinat, int *ukuran, Mat imgThresholded)
{
    std::vector<Rect> ball;
    Mat frame_gray; 

    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect ball
    int xawal;
    Point2D center;
    Point Center;
    int pengkali;

    static int pengkalilama;

    if(pengkalilama > 3 || pengkalilama < 0)
    {
        pengkalilama = 0;
    }

    pengkali = pengkalilama;

    for(pengkali; pengkali < 4; pengkali++)
    {
        xawal = pengkali * 80;
        Rect kotak(xawal, 0, 80, 150);
    //    rectangle(src, kotak, Scalar(255), 1, 8, 0);

        Mat bagian = frame_gray(kotak);
        Mat bagianw = imgThresholdedW(kotak);

        ball_cascade.detectMultiScale(bagian, ball, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE|CV_HAAR_DO_CANNY_PRUNING|CV_HAAR_DO_ROUGH_SEARCH, Size(30, 30) );
        //ball_cascade.detectMultiScale( frame_gray, ball, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE|CV_HAAR_DO_CANNY_PRUNING|CV_HAAR_DO_ROUGH_SEARCH, Size(30, 30) );
      
        int ballsize;
        bool adaputih, adahijau;
        adaputih = false;
        *ukuran = -1;

        //Expectation should have only detect 1 ball or none
        if(ball.size() > 0){
            for(size_t i = 0; i < ball.size(); i++) //maybe try to add ball size limit relative to camera resolution based on ball area in pixel
            {
                for(int k = -10; k <=10; k++ ){
                    for(int j = -10; j <=10; j++ ){
                        if (bagianw.at<uchar>((ball[i].y + ball[i].height*0.5 + k),(ball[i].x + ball[i].width*0.5 + j))==255){
                            adaputih = true;
                        }
                        else {
                            adaputih = false;
                        }                
                    }
                }

                if (imgThresholded.at<uchar>(ball[i].y + ball[i].height*0.5, ball[i].x + ball[i].width*0.5) == 0){
                    adahijau = true;
                }
                else
                {
                    adahijau = false;
                }

                if(adaputih == true && adahijau == true){
                    center.X=ball[i].x + ball[i].width*0.5;
                    center.Y=ball[i].y + ball[i].height*0.5;
                    // Center.x = center.X;
                    // Center.y = center.Y;
                    ballsize = ball[i].height*0.5;
                    // // circle center
                    // circle( frame, Center, 3, Scalar(0,255,0), -1, 8, 0 );
                    // // circle outline
                    // circle( frame, Center, ballsize, Scalar(0,0,255), 3, 8, 0 );
                    *ukuran = ballsize;
                    *koordinat = center;
                    pengkalilama = pengkali;
                }
            }
        }
        else
        {
            center.X = -1;
            center.Y = -1;
        }

        if(*ukuran != -1)
        {
            break;
        }
        pengkalilama = 0;
    }
    
    printf("\nTitik ketemu : %d", ball.size());
    printf("\nKoordinat X : %d", (int)center.X);
    printf("\nKoordinat Y : %d", (int)center.Y);
    // imshow("Result", frame);        

}

