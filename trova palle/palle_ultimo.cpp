#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#define dst 20
#include <stdio.h>
#include<cstdlib>
#include<vector>
#include<queue>
#include<map>
#include<math.h>

#define ii pair<int,int>

using namespace cv;
using namespace std;


//http://www.learnopencv.com/blob-detection-using-opencv-python-c/

Mat tresholda(Mat& imm){
    Mat tbin(imm.rows, imm.cols, CV_8UC1,Scalar(0));

    for(int r =0; r < imm.rows; r++)
        for(int c = 0; c < imm.cols; c++)
        {
            Vec3b intensity = imm.at<Vec3b>(r, c);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];
            //cout<<(int)green<<endl;
            if(green > 100 && blue < 80 && red < 80)
                tbin.at<uchar>(r,c)=(uchar)(255);
        }

        return tbin;
}

int main(int argc, char** argv)
{
    Mat imm,imm2;

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);


SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;

// Filter by Circularity
params.filterByCircularity = false;
params.filterByArea = true;
params.filterByColor = true;
params.filterByConvexity = false;
params.filterByInertia = false;
params.minDistBetweenBlobs = 30;

params.minCircularity = 0.3;
params.maxCircularity = 1.7;
params.blobColor = 255;
params.maxArea = 1000;
params.minArea = 0;

Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
cout<<"STO UTILIZZANDO "<<CV_MAJOR_VERSION<<endl;

for(;;){
for(int contrl=0;contrl<5;contrl++)
    cap >> imm; // get a new frame from camera
    // Setup SimpleBlobDetector parameters.
imshow("roba",imm);
//waitKey(0);
std::vector<KeyPoint> keypoints;

imm = tresholda(imm);


imshow("cacca",imm);
//waitKey(0);

detector->detect( imm, keypoints);

cout<<"ne ho trovati "<<keypoints.size()<<endl;

    drawKeypoints( imm, keypoints, imm2, Scalar(0,250,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imwrite("immagine2.png", imm2);

    imm2 = imread("immagine2.png", CV_LOAD_IMAGE_COLOR);

    for(int i=0; i<keypoints.size(); i++){
    cout<<"HO TROVATO  QUALCOSA a: x"<<(int)keypoints[i].pt.x<<" y "<<(int)keypoints[i].pt.y<<endl;

    circle(imm2,
            Point((int)keypoints[i].pt.x,(int)keypoints[i].pt.y),
            5,
            Scalar(250,0,255));
    }



    imshow("roba",imm2);
    imwrite("immagine2.png", imm2);
    waitKey(0);
   }
    return 0;
}
