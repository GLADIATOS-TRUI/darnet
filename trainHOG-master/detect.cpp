#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int main()
{
    cout << "WAT HAPPEN PLES";

    vector<float> descriptorVector;
    printf("WAT HAPPEN");
//    vector<unsigned int> descriptorVectorIndices;
    // Generate a single detecting feature vector (v1 | b) from the trained support vectors, for use e.g. with the HOG algorithm

//	TRAINHOG_SVM_TO_TRAIN::getInstance()->loadModelFromFile(svmModelFile);

//    TRAINHOG_SVM_TO_TRAIN::getInstance()->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
	int count;
	printf("line 70");
    ifstream myfile ("genfiles/descriptorvector.dat", ios::in);

    float number;  

    myfile >> count;

    for(int i = 0; i < count; i++) {
        myfile >> number;
		printf("Number : %f", number);
        descriptorVector.push_back(number);
    }

	printf("Testing");
    HOGDescriptor hog; // Use standard parameters here
    hog.winSize = Size(64, 64); // Default training images size as used in paper
//	hog.load(cvHOGFile);
//	hog.readALTModel(cvHOGFile);
    hog.setSVMDetector(descriptorVector);
//  hog.setSVMDetector(cvHOGFile);

	double hitThreshold = 1.85;

	printf("HitThreshold : %f", hitThreshold);
    VideoCapture cap(-1); // open the default camera

    if(!cap.isOpened()) { // check if we succeeded
        printf("Error opening camera!\n");
        return EXIT_FAILURE;
    }
    
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
//	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    
    Mat testImage, testing;
	
	char key;
	int a, b;
	a = 1;
	b = 212;

	printf("Testing");
    while (1) 
    {
        cap >> testImage; // get a new frame from camera

		testing = testImage.clone();
		
        //cvtColor(testImage, testImage, CV_BGR2GRAY); // Work on grayscale images as trained

//        detectTest(hog, hitThreshold, testImage);

//        imshow("HOG custom detection", testImage);

		key = cvWaitKey(10);

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
    // </editor-fold>

    return 0;

}
