#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

using namespace cv; 
using namespace std;
int main()
{
 //Load an 
 Mat gray, canny;
 Mat img = imread("eu.jpg", CV_LOAD_IMAGE_COLOR);
 namedWindow("Image", CV_WINDOW_AUTOSIZE);
 imshow("Image", img);
 
cvtColor(img, gray, COLOR_BGR2HSV);
 //Change Contrast Effect
 Mat imgInc;
 img.convertTo(imgInc, -1, 1.5, 0);  //increase (double) contrast 

 Mat imgDec;
 img.convertTo(imgDec, -1, 0.5, 0); //decrease (halve) contrast

 namedWindow("Inc contrast", CV_WINDOW_AUTOSIZE);
 namedWindow("Dec contrast", CV_WINDOW_AUTOSIZE);

 imshow("Inc contrast", gray);
 //imshow("branco", img);
//imshow("branco", canny);
 //Wait Key press
 cvWaitKey(0);

 //destroy windows
 destroyAllWindows();
 return 0;
}