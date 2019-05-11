
#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <string> 
#include <stdlib.h>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
 
using namespace cv; 
using namespace std;

int fd;
int servoMinPos = 800;
int servoMaxPos = 2000;
int servoMinCh = 0;
int servoMaxCh = 31;


// Gabriel ===================================================>

//Convert integer to string
std::string int_to_string(int n)
{
  std::ostringstream str1;
  str1 << n;
  std::string str = str1.str();
  return str1.str();
}

int verifyBounds(int ch, int pos)
{
  if (ch > servoMaxCh or ch < servoMinCh)
  {
    std::cout << "Channel is not within its boundaries\n";
    return 0;
  }
  if (pos > servoMaxPos or pos < servoMinPos)
  {
    std::cout << "Position is not within its boundaries\n";
    return 0;
  }
  return 1;
}

int openPort(void)
{
  int fd; /* File descriptor for the port */
  struct termios options;
  int baud = B115200;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
  {
    perror("openPort: Unable to open /dev/ttyUSB0\n"); //It coud not open port
  }
  else
    fcntl(fd, F_SETFL, 0);

  memset(&options, 0, sizeof(options));
  cfsetispeed( &options, baud );
  cfsetospeed( &options, baud );

  options.c_iflag = IGNBRK | IGNPAR;
  options.c_oflag = 0;
  options.c_cflag |= CREAD | CS8 | CLOCAL;
  options.c_lflag = 0;

  if( tcsetattr( fd, TCSANOW, &options ) < 0 )
  {
    printf( "ERROR: setting termios options\n" );
    return false;
  }

  // Make sure queues are empty
  tcflush( fd, TCIOFLUSH );

  printf( "Successfully opened port /dev/ttyUSB0\n");

  return (fd);
}

// Move the ch servo to postition pos
void moveServo(int ch, int pos)
{
  if(verifyBounds(ch, pos) == 1)
  {
    int n;
    std::string str = '#' + int_to_string(ch) + " P" + int_to_string(pos) + '\r';
    const char * msg = str.c_str();
    n = write(fd, msg, strlen(msg));
    if (n < 0)
      fputs("write() failed!\n", stderr);
  }
}

// Centralizes all servos
void setServos(int num) {
  int NUM_ACTIVE_SERVOS = num;

  for(int i = 1; i <= NUM_ACTIVE_SERVOS; i++) {
    moveServo(i-1, 1500);
    sleep(0.1); //give each servo initial wait time
  }
}

//=============================================================>

//função para ler posição do objeto //

int* ler_posicao_objeto(Mat src, int ant_x, int ant_y){
    int* v_o = new int[2];

    v_o[0] = ant_x;
    v_o[1] = ant_y;

    Mat canny, hsv, black;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //cvtColor(src, hsv, COLOR_BGR2HLS);
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
        v_o[0] = mc[i].x;
        v_o[1] = mc[i].y; 

      }
    }
    
    cout << "oX = " << v_o[0] << "oY = "<< v_o[1] << endl;
    imshow("Contour", src);
   return v_o;
}

// função para ler posição do laser//
int* ler_posicao_laser(Mat src, int ant_lx, int ant_ly ){
    int* v_l = new int[2];
    v_l[0] = ant_lx;
    v_l[1] = ant_lx;

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
      mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // desenha

    Mat drawing(canny.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < contours.size(); i++){
        Scalar color1 = Scalar(0, 255, 0); // B G R values
       // circle(src, mc[i], 2, color1, -1, 8, 0);
        v_l[0] = (int)mc[i].x;
        v_l[1] = (int)mc[i].y;
    }
    //cout << " X = " << v_l[0] << " Y = "<< v_l[1] << endl;
     //imshow("laser", src);
     return v_l;
}

//função que calcula erro e retorna largura de pulso
int* calcula_pulso_pwm(int o_x, int o_y, int l_x, int l_y, int pwm_x, int pwm_y){
     // int x = 1, y = 1;
      int* pwm_atual = new int[2];
      pwm_atual[0] = pwm_x;
      pwm_atual[1] = pwm_y;
      int erro[2];
      erro[0] = o_x - l_x;
      erro[1] = o_y - l_y;
     // cout <<erro[0]<<"x" << erro[1]<<endl;

      
      if(erro[0] >= 8){
         pwm_atual[0] = pwm_x - 1;
         if(pwm_atual[0] < 1300){
           pwm_atual[0] = 1300;
         }
      }
      if(erro[0] < -8){
         pwm_atual[0] = pwm_x + 1;
         if(pwm_atual[0] >  1700){
           pwm_atual[0] = 1700;
         }
      }
      if(erro[1] >= 8){
         pwm_atual[1] = pwm_y - 1;
         if(pwm_atual[1] < 1300){
           pwm_atual[1] = 1300;
         }
      }
      if(erro[1] < -8 ){
         pwm_atual[1] = pwm_y + 1;
         if(pwm_atual[1] > 1700){
           pwm_atual[1] = 1700;
         }
      }
      //  cout << pwm_x << "x"<< pwm_y << endl;
      return pwm_atual;

}
// Função que aciona os servos motores
void acionar_servo(int pwm_x, int pwm_y){
  //cout << pwm_x << "*" << pwm_y << endl;
  moveServo(0,pwm_x);
  moveServo(1,pwm_y);
   
}
// função principal //
int main(){
  int fd1 = openPort();
  fd = fd1;
     setServos(2);
    // setup da webcam.
    VideoCapture cap(1);
    //Verificação se a camera foi aberta. 
    if(!cap.isOpened()){
         cout << "Erro ao abrir a camera" << endl;
         return -1;
      }
  // Definição das dimenssões do frame capturado.
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  // Declaração das variáveis para receber a posição do laser e do objeto e o offset dos servo motores.    
  int laser_x, laser_y;
  int objeto_x = 200, objeto_y = 200;
  int pwm_x = 1400;
  int pwm_y = 1400;
  // Declaração dos ponteiros para as função que captura as posições do objeto, do laser, e largura do pwm. 
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
   v_objeto = ler_posicao_objeto(src, objeto_x, objeto_y);
   v_laser = ler_posicao_laser(src, laser_x, laser_y);
   //Coordenada do objeto
   objeto_x = v_objeto[0];
   objeto_y = v_objeto[1];
   //Coordenada do 
   laser_x = v_laser[0];
   laser_y = v_laser[1];
   // Função para calcular o erro e tranforma em largura de pulso
   pwm = calcula_pulso_pwm(objeto_x, objeto_y, laser_x, laser_y, pwm_x, pwm_y);
   // valores de de lagura de pulso para os servos motores 
   pwm_x = pwm[0];
   pwm_y = pwm[1];
   // Função para acionar os dois servo motores
   acionar_servo(pwm_x, pwm_y);

  //precione ESC para sair
  char c = (char)waitKey(1);
    if (c == 27)
      break;
  }
  // fecha a porta serial
  close(fd);
//Fecha a janela
  destroyAllWindows();
  return 0;
}