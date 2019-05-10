#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace std;
using namespace cv;

int x = 0;

int main(int argc, char const *argv[])
{
    char* imgs_directory;
    char* extension;
    int im_width, im_height;

    static struct poptOption options[] = {
        { "img_width",'w',POPT_ARG_INT,&im_width,0,"Image width","NUM" },
        { "img_height",'h',POPT_ARG_INT,&im_height,0,"Image height","NUM" },
        { "imgs_directory",'d',POPT_ARG_STRING,&imgs_directory,0,"Directory to save images in","STR" },
        { "extension",'e',POPT_ARG_STRING,&extension,0,"Image extension","STR" },
        POPT_AUTOHELP
        { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    VideoCapture cap1(1);
    VideoCapture cap2(2);
    Mat img1, img_res1, img2, img_res2;
    int ch=0;
    int n = (ch & 0xFF);
    bool ok_l, ok_r;
    cout << "Starting, ch=" << ch << ", n=" << n << endl;
    while ((ch & 0xFF) != 27) {
        ok_l = cap1.grab();
        ok_r = cap2.grab();

        if (!ok_l || !ok_r) {
          cout << "ERROR grabbing image" << endl;
          break;
        }
        cap1.retrieve(img1);
        cap2.retrieve(img2);
        resize(img1, img_res1, Size(im_width, im_height));
        resize(img2, img_res2, Size(im_width, im_height));
        imshow("IMG1", img_res1);
        imshow("IMG2", img_res2);
        moveWindow("IMG1", 10, 10);
        moveWindow("IMG2", 680, 10);
        ch = waitKey(30);
        if ((ch & 0xFF) == 32) {
            x++;
            char filename1[200], filename2[200];
            sprintf(filename1, "%sleft%02d.%s", imgs_directory, x, extension);
            sprintf(filename2, "%sright%02d.%s", imgs_directory, x, extension);
            cout << "Saving img pair " << x << ", left: " << filename1 << ", right: " << filename2 << endl;
            imwrite(filename1, img_res1);
            imwrite(filename2, img_res2);
        }
    }
    cout << "DONE" << endl;
    cap1.release();
    cap2.release();
    destroyAllWindows();
    return 0;
}
