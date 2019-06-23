#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace cv; 
using namespace std;

int main(){
    Mat src = imread("teste.jpg",1);

Mat hsv, canny, black;

    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(165, 98, 121), Scalar(180, 255, 255), black);

    Canny(black, canny, 50, 150, 3);


imshow( "origial",src);
imshow( "hsv",hsv);
imshow( "black",black);
imshow( "canny",canny);
waitKey(0);

    waitKey(0);

    return 0;
}