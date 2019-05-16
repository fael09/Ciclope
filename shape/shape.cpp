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

      VideoCapture cap("video.mp4");    

 if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }


     while(1){
 
    Mat src;
    cap >> src;
    if (src.empty())
      break;

  Mat src_gray;


  //src = imread("branco.jpeg", 1 );

  if( !src.data )
    { return -1; }

 
  cvtColor( src, src_gray, CV_BGR2GRAY );


  GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;


  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 100, 50, 18, 20 );
  // cout << circle << endl;

  for(  int i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);

     // cout << i << endl;
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );

      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }


    imshow( "Frame", src );
 
    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}