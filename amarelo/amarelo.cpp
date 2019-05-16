#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
 
using namespace std;
using namespace cv;
 
int main(){
 
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap("video.mp4"); 
    
  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
     
  while(1){
 
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
 
    // Display the resulting frame
    // convert to grayscale (you could load as grayscale instead)
    Mat gray, mask, canny, black;
    cvtColor( frame, gray, COLOR_BGR2HSV);
     //cvtColor( input, gray, COLOR_BGR2GRAY);
    // Canny(gray, mask, 0, 20, 3);

    inRange(gray,Scalar(10,130,160),Scalar(255,255,255),mask ); // cor amarela 
    //inRange(gray,Scalar(0,0,0),Scalar(80,80,80),mask ); // branco
   // threshold(gray, mask, 200, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //Canny(mask, canny_output, 0, 255, 3);
    findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = 0;
    float biggestContourArea = 0;
    Mat drawing(mask.size(), CV_8UC3, Scalar(0, 0, 0));
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar(0, 0, 255);
        drawContours( frame, contours, i, color, 5, 8, hierarchy, 0, Point() );


        float ctArea= contourArea(contours[i]);
        if(ctArea > biggestContourArea)
        {
            biggestContourArea = ctArea;
            biggestContourIdx = i;
        }
    }

    // if no contour found
    if(biggestContourIdx < 0)
    {
        cout << "no contour found" << endl;
        return 1;
    }

    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
  
    //
    
    


    imshow( "Frame", frame );
 
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