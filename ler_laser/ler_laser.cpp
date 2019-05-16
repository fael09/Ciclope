#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv; 
using namespace std;

int main(){
//Mat src = imread("laser.jpg",1);
    VideoCapture cap(1);

      if (!cap.isOpened())
       {
         cout << "Erro ao abrir a camera" << endl;
         return -1;
      }

  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  while (1){

Mat src, hsv, canny, black;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

  cap >> src;

   if (src.empty())
    break;

    //cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(src, Scalar(220,212, 240), Scalar(255, 255, 255), black);


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

    imshow("Contour", src);
    //precione ESC para sair
    char c = (char)waitKey(1);
    if (c == 27)
      break;
  }
//Fecha a janela
  destroyAllWindows();
  return 0;
}