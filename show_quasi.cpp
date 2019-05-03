#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/stereo.hpp>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "popt_pp.h"

using namespace cv;
using namespace std;

static void quasi(Mat& leftImg, Mat& rightImg, Mat& disp)
{
    cout << "quasi called, leftImg size: " << leftImg.size[0] << "/" << leftImg.size[1] << endl;
    cv::Size frameSize = leftImg.size();
    Ptr<stereo::QuasiDenseStereo> stereo = stereo::QuasiDenseStereo::create(frameSize);

    stereo->process(leftImg, rightImg);

    uint8_t displvl = 80;

    disp = stereo->getDisparity(displvl);
}

int main(int argc, char** argv)
{

    char* leftimg_filename;
    char* rightimg_filename;
    char* calib_file;
    char* leftout_filename;
    char* rightout_filename;

    static struct poptOption options[] = {
      { "leftimg_filename",'l',POPT_ARG_STRING,&leftimg_filename,0,"Left imgage path","STR" },
      { "rightimg_filename",'r',POPT_ARG_STRING,&rightimg_filename,0,"Right image path","STR" },
      { "calib_file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
      { "leftout_filename",'L',POPT_ARG_STRING,&leftout_filename,0,"Left undistorted imgage path","STR" },
      { "rightout_filename",'R',POPT_ARG_STRING,&rightout_filename,0,"Right undistorted image path","STR" },
      POPT_AUTOHELP
      { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    Mat R1, R2, P1, P2, Q;
    Mat K1, K2, R;
    Vec3d T;
    Mat D1, D2;

    cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
    fs1["K1"] >> K1;
    fs1["K2"] >> K2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;

    fs1["R1"] >> R1;
    fs1["R2"] >> R2;
    fs1["P1"] >> P1;
    fs1["P2"] >> P2;
    fs1["Q"] >> Q;

    Mat img_orig_left = imread(leftimg_filename, IMREAD_COLOR);
    Mat img_orig_right = imread(rightimg_filename, IMREAD_COLOR);

    if (img_orig_left.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img_orig_right.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

    cv::Mat lmapx, lmapy, rmapx, rmapy;
    cv::Mat img_fixed_left, img_fixed_right;

    cv::initUndistortRectifyMap(K1, D1, R1, P1, img_orig_left.size(), CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, img_orig_right.size(), CV_32F, rmapx, rmapy);
    cv::remap(img_orig_left, img_fixed_left, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(img_orig_right, img_fixed_right, rmapx, rmapy, cv::INTER_LINEAR);

    Mat img_rot_left, img_rot_right;
    rotate(img_fixed_left, img_rot_left, ROTATE_180);
    rotate(img_fixed_right, img_rot_right, ROTATE_180);

    imwrite(leftout_filename, img_rot_left);
    imwrite(rightout_filename, img_rot_right);

    cout << "created left and right images, creating disparity" << endl;

    Mat disp, disp8;

    int i = 0;
    int64 t = getTickCount();
    for (i = 0; i < 5; i++) {
        t = getTickCount();

        quasi(img_rot_left, img_rot_right, disp);
    }
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    disp.convertTo(disp8, CV_8U);

    namedWindow("left", 1);
    imshow("left", img_rot_left);
    moveWindow("left", 5, 5);
    namedWindow("right", 1);
    imshow("right", img_rot_right);
    moveWindow("right", 800, 5);
    namedWindow("disparity", 0);
    imshow("disparity", disp8);
    moveWindow("disparity", 5, 500);
    printf("press any key to continue...");
    fflush(stdout);
    waitKey();
    printf("\n");

    return 0;
}
