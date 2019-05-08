
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <iostream>
#include "popt_pp.h"

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{

    char* leftimg_filename;
    char* rightimg_filename;
    char* calib_file;
    int   view_flag = 0;
    int   numberOfDisparities;
    int SADWindowSize;
    float scale;

    static struct poptOption options[] = {
        { "leftimg_filename",  'l', POPT_ARG_STRING, &leftimg_filename,    0,"Left imgage path","STR" },
        { "rightimg_filename", 'r', POPT_ARG_STRING, &rightimg_filename,   0,"Right image path","STR" },
        { "calib_file",        'c', POPT_ARG_STRING, &calib_file,          0,"Stereo calibration file","STR" },
        { "view",              'v', POPT_ARG_NONE,   &view_flag,           0, "View image", NULL },
        { "max_disp",          'd', POPT_ARG_INT,    &numberOfDisparities, 0, "Max Number of disparities", NULL },
        { "block_size",        'b', POPT_ARG_INT,    &SADWindowSize,       0, "Block Size", NULL },
        { "scale",             's', POPT_ARG_FLOAT,  &scale,               1, "Scale", NULL },
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


    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);

    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }

    Rect roi1, roi2;
    Mat imgL, imgR;

    K1 *= scale;
    K2 *= scale;

    Size img_size = Size(640, 480);
    stereoRectify( K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    Mat imgLr, imgRr;

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = 3;

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_SGBM);

    Mat disp, disp8;

    VideoCapture leftVid(leftimg_filename);
    VideoCapture rightVid(rightimg_filename);

    if (!leftVid.isOpened() || !rightVid.isOpened()) {
        cout << "Error opening video" << endl;
        exit(-1);
    }

    int ch = 0;
    bool leftOk, rightOk;

    while (ch != 27) {

        leftOk = leftVid.grab();
        rightOk = rightVid.grab();
        if (!leftOk || !rightOk) {
            cout << "Error grabbing frame" << endl;
            break;
        }

        leftVid.retrieve(imgL);
        rightVid.retrieve(imgR);

        int64 t = getTickCount();
        remap(imgL, imgLr, map11, map12, INTER_LINEAR);
        remap(imgR, imgRr, map21, map22, INTER_LINEAR);

        sgbm->compute(imgLr, imgRr, disp);
        t = getTickCount() - t;
        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

        disp.convertTo(disp8, CV_8U);

        if( view_flag )
        {
            namedWindow("left", 1);
            imshow("left", imgL);
            namedWindow("right", 1);
            imshow("right", imgR);
            namedWindow("disparity", 0);
            imshow("disparity", disp8);
            printf("press any key to continue...");
            fflush(stdout);
            ch = waitKey(1) & 0xFF;
            printf("\n");
        }
    }

    leftVid.release();
    rightVid.release();

    destroyAllWindows();

    return 0;
}
