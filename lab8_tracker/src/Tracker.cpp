#include "stdafx.h"
#include "Tracker.h"
#define COM_PORT 9
#define WRITE_FILE 4000 // how often to write image to file, in milliseconds

Mat Tracker::hsvFilter(const Mat &src, int hueLow, int hueHigh)
   {
   Mat result;
   inRange(src, Scalar(hueLow, 110, 110), Scalar(hueHigh, 255, 255), result);

   return result;
   }

Tracker::Tracker()
   {
   MCU = new Control(COM_PORT);	
   Server.init(4315);
   Server.server_start();
   namedWindow("Video");
   namedWindow("Threshold");
   }

Tracker::~Tracker()
   {
   Server.server_stop();
   };

void Tracker::Update(){
   VideoCapture vid;
   bool exit = false;
   bool laserOn = true;
   bool tracking = true;
   int writeTime = GetTickCount();

   vid.open(1);					// Open webcam 1 (not 0 since that is my laptop's internal webcam)
   MCU->set_data('0', '4', laserOn);

   if (vid.isOpened() == TRUE)
      {
      while(!exit)
         {
         vid.read(imvid);								// read the webcam image
         cvtColor(imvid, imthresh, CV_BGR2HSV);			// convert webcam image to HSV and store in imthresh

         imthresh = hsvFilter(imthresh, 90, 180);

         std::vector<std::vector<cv::Point> > contours;	// create a vector object to hold the contours
         Mat cont = imthresh.clone();					// make a copy of the threshold image

         findContours(cont, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // find all the contours in the image
         Rect ball = Rect(0,0,0,0);						// create a rectangle for the ball with 0 size

         for (size_t idx = 0; idx < contours.size(); idx++)				// loop through all the contours
            {
            Rect r = boundingRect(contours.at(idx));					// make a bounding rect around the current contour
            drawContours(imvid, contours, idx, Scalar(255,255,255));	// draw the current contour

            if (r.width * r.height > ball.width * ball.height)			// if the current rect is bigger than the ball, save this rect to ball
               ball = r;
            }

         // draw a rectangle around the biggest blue object
         rectangle(imvid, Point(ball.x, ball.y), Point(ball.x + ball.width, ball.y + ball.height), Scalar(255,255,255));
         int targetX = ball.x + ball.width / 2;		// calculate the coordinate of the middle of the ball
         int targetY = ball.y + ball.height / 2;

         if (ball.width * ball.height > 300)
            {// if the area of the ball is large enough, draw a line indicating where the targetX is
            int servoX = ((float)targetX / (float)imvid.cols * 59) + 93;

            int servoY = (((float)targetY / (float)imvid.rows) * 48) + 83;

            line(imvid, Point(targetX, 0), Point(targetX, imvid.rows), Scalar(0,0,255));
            line(imvid, Point(0, targetY), Point(imvid.cols, targetY), Scalar(0,0,255));

            if (tracking)
               {
               MCU->set_data('3', '0', servoX);
               MCU->set_data('3', '1', servoY);
               }

            //printf("X, Y: %i, %i\n", servoX, servoY);
            }

         if (!imvid.empty())
            {
            if (GetTickCount() - writeTime >= WRITE_FILE)
               {
               imwrite("..\\lab10 client\\client\\debug\\cam.jpg", imvid);
               writeTime = GetTickCount();
               }
            imshow ("Threshold", imthresh);			// display a window with the threshold image for debugging
            imshow ("Video", imvid);				// display a window with the webcam image overlayed with the targetX indicators
            }
         Server._mutex.lock();
         laserOn = Server._laser;
         tracking = Server._tracking;
         Server._mutex.unlock();

         switch (waitKey(10))
            {
            case 'q':
               exit = true;
               break;
            case 'l':
               laserOn = !laserOn;

               break;
            case 't':
               tracking = !tracking;
               break;

            }


         MCU->set_data('0', '4', laserOn);

         }

      }      
   }