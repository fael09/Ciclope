#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv; 
using namespace std;


//função para ler posição do objeto //

int* ler_posicao_objeto(Mat src){
    int* v_o = new int[2];
    
    Mat canny, hsv, black;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 140, 140), Scalar(255, 255, 255), black);
    Canny(black, canny, 50, 150, 3);
    findContours(canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    Mat drawing1(canny.size(), CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < contours.size(); i++)
    {
      Scalar color3 = Scalar(255, 255, 255);
      drawContours(drawing1, contours, i, color3, 2, 2, hierarchy, 0, Point());
    }

    cvtColor(drawing1, black, COLOR_BGR2HSV);
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
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }
     
    // desenha

    Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < contours.size(); i++)
    {
      double a = contourArea(contours[i], false);

      if (a > 2000)
      {
        Scalar color2 = Scalar(255, 0, 0); // B G R values
        Scalar color1 = Scalar(0, 255, 0); // B G R values
        drawContours(src, contours, i, color1, 2, 8, hierarchy, 0, Point());
        circle(src, mc[i], 5, color2, -1, 8, 0);
        v_o[0] = (int)mc[i].x;
        v_o[1] = (int)mc[i].y;
      
      }
    }
    //imshow("objeto", src);
    return v_o;
}

// função para ler posição do laser//
int* ler_posicao_laser(Mat src){
    int* v_l = new int[2];

    Mat canny, black;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
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
    for (int i = 0; i < contours.size(); i++){
        Scalar color1 = Scalar(0, 255, 0); // B G R values
        circle(src, mc[i], 2, color1, -1, 8, 0);
        v_l[0] = (int)mc[i].x;
        v_l[1] = (int)mc[i].y;
    }
     //imshow("laser", src);
     return v_l;
}

//função que calcula erro e retorna largura de pulso
int* calcula_pulso_pwm(int o_x, int o_y, int l_x, int l_y, int pwm_x, int pwm_y){
      int* pwm_atual = new int[2];
      int erro[2];
      erro[0] = o_x - l_x;
      erro[1] = o_y - l_y;

      if(erro[0] > 0){
         pwm_atual[0] = pwm_x - 1;
      }
      if(erro[0] < 0){
         pwm_atual[0] = pwm_x + 1;
      }
      if(erro[1] > 0){
         pwm_atual[0] = pwm_y - 1;
      }
      if(erro[1] < 0){
         pwm_atual[0] = pwm_y + 1;
      }

      return pwm_atual;

}

// função principal //
int main(){
    // setup da webcam.
    VideoCapture cap(0);
    //Verificação se a camera foi aberta. 
      if (!cap.isOpened())
       {
         cout << "Erro ao abrir a camera" << endl;
         return -1;
      }
  // Definição das dimenssões do frame capturado.
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  // Declaração das variáveis para receber a posição do laser e do objeto.    
  int laser_x, laser_y;
  int objeto_x, objeto_y;
  int pwm_x = 690;
  int pwm_y = 690;
  // Declaração dos ponteiros para as função que captura as posições do objeto e do laser. 
  int* v_laser;
  int* v_objeto;
  int* pwm;
  // loop para captura dos frames.
  while (1){
    // declaração da imagem de entrada src.
    Mat src;
    cap >> src;
   // verificação se a imagem tem conteudo.
   if (src.empty()){
    break;
   }
   //Funções para captura das posições, tanto do laser quanto do objeto.
   v_objeto = ler_posicao_objeto(src);
   v_laser = ler_posicao_laser(src);
   //Coordenada do objeto
   objeto_x = v_objeto[0];
   objeto_x = v_objeto[1];
   //Coordenada do 
   laser_x = v_laser[0];
   laser_y = v_laser[1];
  // Função para calcular o erro e tranforma em largura de pulso
  pwm = calcula_pulso_pwm(objeto_x, objeto_y, laser_x, laser_y, pwm_x, pwm_y);
  // valores de de lagura de pulso para os servos motores 
  pwm_x = pwm[0];
  pwm_y = pwm[1];


   //Cout mostra na tela as posições do objeto e do laze: (x,y)
   cout <<"Objeto = " << objeto_x << ","<< objeto_y <<
    " Laser = " << laser_x << "," << laser_y << endl;
   
    //precione ESC para sair
    char c = (char)waitKey(1);
    if (c == 27)
      break;
  }
//Fecha a janela
  destroyAllWindows();
  return 0;
}