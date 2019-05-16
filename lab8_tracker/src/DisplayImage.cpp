#include <stdio.h>
#include <opencv2 / opencv.hpp>
usando namespace cv ;
int main ( int argc, char ** argv)
{
    if (argc! = 2)
    {
        printf ( "uso: DisplayImage.out <Image_Path> \ n" );
        return -1;
    }
    Imagem da esteira ;
    imagem = imread (argv [1], 1);
    if (! image. data )
    {
        printf ( "Sem dados de imagem \ n" );
        return -1;
    }
    namedWindow ( "Display Image" , WINDOW_AUTOSIZE );
    imshow ( "Display Image" , imagem);
    waitKey (0);
    return 0;
}