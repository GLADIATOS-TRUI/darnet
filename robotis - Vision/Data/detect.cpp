#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
//#include <cv.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int main()
{
//    vector<unsigned int> descriptorVectorIndices;
    // Generate a single detecting feature vector (v1 | b) from the trained support vectors, for use e.g. with the HOG algorithm

//	TRAINHOG_SVM_TO_TRAIN::getInstance()->loadModelFromFile(svmModelFile);

//    TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
	int count;
	
	printf("Testing\n");
    HOGDescriptor hog; // Use standard parameters here
//    hog.winSize = Size(64, 64); // Default training images size as used in paper
	hog.load("cvHOGClassifier.yaml");
//	hog.readALTModel(cvHOGFile);
//    hog.setSVMDetector(descriptorVector);
//  hog.setSVMDetector(cvHOGFile);

	double hitThreshold = 1.8375034;

	printf("HitThreshold : %f\n", hitThreshold);
    VideoCapture cap(-1); // open the default camera

    if(!cap.isOpened()) { // check if we succeeded
        printf("Error opening camera!\n");
        return EXIT_FAILURE;
    }
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    
    Mat testImage, testing;
	
	char key;
	int a, b;
	a = 1;
	b = 212;

	printf("Testing\n");
    while (1) 
    {
        cap >> testImage; // get a new frame from camera

		testing = testImage.clone();
		
        //cvtColor(testImage, testImage, CV_BGR2GRAY); // Work on grayscale images as trained

//        detectTest(hog, hitThreshold, testImage);
		vector<Rect> ball;
		Point Center;
		
		Size padding(Size(8, 8));
		Size winStride(Size(8, 8));
		
		hog.detectMultiScale(testImage, ball, hitThreshold, winStride, padding, 1.1);

		int ballsize = 0;
        for(size_t i = 0; i < ball.size(); i++) //maybe try to add ball size limit relative to camera resolution based on ball area in pixel
        {

			Center.x = ball[i].x + ball[i].width*0.5;
			Center.y = ball[i].y + ball[i].height*0.5;
			ballsize = ball[i].height*0.5;
			// // circle center
			 circle( testImage, Center, 3, Scalar(0,255,0), -1, 8, 0 );
			// // circle outline
			 circle( testImage, Center, ballsize, Scalar(0,0,255), 3, 8, 0 );

		}
		
        imshow("HOG custom detection", testImage);

		key = cvWaitKey(50);

		if (char(key) == 'a'){
			char filename[80];
			sprintf(filename,"datapos/test_%d.png",a);
			imwrite(filename, testing);
			a++;
		}

		if (char(key) == 's'){
			char filename[80];
			sprintf(filename,"dataneg/test_%d.png",b);
			imwrite(filename, testing);
			b++;
		}

		if(key == 'x')
		{
			break;
		}
    }

    return 0;

}
