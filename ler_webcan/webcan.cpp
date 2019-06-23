#include "opencv2/opencv.hpp"
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <unistd.h>

using namespace std;
using namespace cv;

//-----------------------------------------------------------------
//temporizado = uma estrurura para contar tempo de execução de funções 
struct
{
  struct timeval time;
  struct timeval timereset;
} tictocctrl;

void tic(void)
{
  gettimeofday(&tictocctrl.timereset, NULL);
}

double toc(void)
{
  gettimeofday(&tictocctrl.time, NULL);
  return ((tictocctrl.time.tv_sec - tictocctrl.timereset.tv_sec) + (tictocctrl.time.tv_usec - tictocctrl.timereset.tv_usec) * 1e-6);
}
//---------------------------------------------------------------------------------

int main()
{
//--------------------Funções pardrão para ler dados da webcam-----------------------------
  
  // cria um objeto de captura de vídeo e usa a webcam do pc 
  VideoCapture cap(0);

  // checa se a camera foi aberta corretamente
  if (!cap.isOpened())
  {
    cout << "Erro ao abrir a camera" << endl;
    return -1;
  }

  // Resolução padrão dos frame de imagens-->captura a largura e altura
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  while (1)
  {
    // usleep(1000000);
    Mat src;
      
    // capturar a imagem

    cap >> src;

    // se o frame estiver vazio o programa é parado
    if (src.empty())
      break;

    //---------------------------------------------------
    //Declaração de variáveis 
    Mat canny, hsv, black,erosion, dilation;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
       
  
   // cvtColor(src, hsv, COLOR_BGR2HLS);
    inRange(src, Scalar(0,0, 0), Scalar(255, 95, 255), black);
    Canny(black, canny, 50, 150, 3);

    findContours(canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));


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
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // desenha
  
    Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
      
    for (int i = 0; i < contours.size(); i++)
    {
     double a = contourArea(contours[i], false);
     //cout << a << endl;
      if (a > 500)
      {
        Scalar color2 = Scalar(255, 0, 0); // B G R values
        Scalar color1 = Scalar(0, 255, 0); // B G R values
        drawContours(src, contours, i, color1, 2, 8, hierarchy, 0, Point());
        circle(src, mc[i], 5, color2, -1, 8, 0);
         

      }
    }

    

    imshow("ero", canny);
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