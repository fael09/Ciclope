#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv; 
using namespace std;

int main(){
    Mat src = imread("laser3.jpg",1);

Mat hsv, canny, black;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

    //cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(src, Scalar(180, 180, 240), Scalar(255, 255, 255), black);

    Canny(black, canny, 50, 150, 3);

    findContours(canny, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // pega o momento
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)  
    {
      mu[i] = moments(contours[i], false);
    }

    // pega o centro do objeto.
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
      double a = contourArea(contours[i], false);
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }


    // desenha

    Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < contours.size(); i++)
    {
      double a = contourArea(contours[i], false);
      if (a > 0)
      {
 
        Scalar color1 = Scalar(0, 255, 0); // B G R values

        circle(src, mc[i], 2, color1, -1, 8, 0);
        float x = mc[i].x;
        float y = mc[i].y;
        cout << "Coordenada X = " << x << " Coordenada Y = "<< y << endl;
      }
    }

imshow( "origial",src);
//imshow( "hsv",hsv);
imshow( "black",black);
imshow( "canny",canny);
waitKey(0);

    waitKey(0);

    return 0;
}