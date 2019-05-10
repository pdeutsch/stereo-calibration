#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "popt_pp.h"

using namespace cv;
using std::cout;

const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;
Mat src1;
Mat src2;
Mat dst;
Scalar line_color(0, 0, 255);

static void on_trackbar( int, void* )
{
    alpha = (double) alpha_slider/alpha_slider_max ;
    beta = ( 1.0 - alpha );
    addWeighted( src1, alpha, src2, beta, 0.0, dst);
    for (int y = 0; y < 480; y += 48) {
        Point pt1(0, y);
        Point pt2(639, y);
        line(dst, pt1, pt2, line_color, 1, 8, 0);
    }
    imshow( "Linear Blend", dst );
}

int main( int argc, char const *argv[] )
{
    char* leftimg_filename;
    char* rightimg_filename;

    static struct poptOption options[] = {
      { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left imgage path","STR" },
      { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image path","STR" },
      POPT_AUTOHELP
      { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    src1 = imread(leftimg_filename);
    src2 = imread(rightimg_filename);
    if( src1.empty() ) { cout << "Error loading src1 \n"; return -1; }
    if( src2.empty() ) { cout << "Error loading src2 \n"; return -1; }
    alpha_slider = 0;

    namedWindow("Linear Blend", WINDOW_AUTOSIZE); // Create Window
    char TrackbarName[50];
    sprintf( TrackbarName, "Alpha x %d", alpha_slider_max );
    createTrackbar( TrackbarName, "Linear Blend", &alpha_slider, alpha_slider_max, on_trackbar );
    on_trackbar( alpha_slider, 0 );
    waitKey(0);
    return 0;
}
