import sys
import cv2
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-l", "--left", required=True,            help="left image file")
ap.add_argument("-r", "--right", required=True,            help="right image file")
ap.add_argument("-c", "--calib", required=True,            help="calibration file")
args = vars(ap.parse_args())

fnameL = args['left']
fnameR = args['right']
calib_file = args['calib']

fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)

imgL = cv2.imread(fnameL, cv2.IMREAD_COLOR)
imgR = cv2.imread(fnameR, cv2.IMREAD_COLOR)

M1 = fs.getNode('K1').mat()
M2 = fs.getNode('K2').mat()
D1 = fs.getNode('D1').mat()
D2 = fs.getNode('D2').mat()
R1 = fs.getNode('R1').mat()
R2 = fs.getNode('R2').mat()
P1 = fs.getNode('P1').mat()
P2 = fs.getNode('P2').mat()
fs.release()
print("P2 shape: {}".format(P2.shape))

img_size = (640, 480)
(map11, map12) = cv2.initUndistortRectifyMap(M1, D1, R1, P1, img_size, cv2.CV_16SC2)
(map21, map22) = cv2.initUndistortRectifyMap(M2, D2, R2, P2, img_size, cv2.CV_16SC2)


#     Mat img1r, img2r;
imgL1 = cv2.remap(imgL, map11, map12, cv2.INTER_LINEAR)
imgR1 = cv2.remap(imgR, map21, map22, cv2.INTER_LINEAR)



# 
#     numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
# 
#     sgbm->setPreFilterCap(63);
#     int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
#     sgbm->setBlockSize(sgbmWinSize);
# 
#     int cn = img1.channels();
# 
#     sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
#     sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
#     sgbm->setMinDisparity(0);
#     sgbm->setNumDisparities(numberOfDisparities);
#     sgbm->setUniquenessRatio(10);
#     sgbm->setSpeckleWindowSize(100);
#     sgbm->setSpeckleRange(32);
#     sgbm->setDisp12MaxDiff(1);
#     if(alg==STEREO_HH)
#         sgbm->setMode(StereoSGBM::MODE_HH);
#     else if(alg==STEREO_SGBM)
#         sgbm->setMode(StereoSGBM::MODE_SGBM);
# 

# for y in range(0, 480, 24):
#     cv2.line(imgL2, (0, y), (639, y), (0,0,255), 1, cv2.LINE_8, 0)
#     cv2.line(imgR2, (0, y), (639, y), (0,0,255), 1, cv2.LINE_8, 0)
# 
# cv2.imshow('ORIG', img)
# cv2.moveWindow('ORIG', 10, 10)
# cv2.imshow('FIXED', img2)
# cv2.moveWindow('FIXED', 680, 10)
# 
# cv2.waitKey(0)
# 
# 
# 
# 
# 
# 
# 
# 
# 
# int main(int argc, char** argv)
# {
#     enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
#     int alg = STEREO_SGBM;
#     int SADWindowSize, numberOfDisparities;
#     bool no_display;
#     float scale;
# 
#     Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
# 
#     numberOfDisparities = parser.get<int>("max-disparity");
#     SADWindowSize = parser.get<int>("blocksize");
#     scale = parser.get<float>("scale");
#     no_display = parser.has("no-display");
# 
# 
#     if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 ) {
#         printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
#     }
#     if (scale < 0) {
#         printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
#     }
#     if (SADWindowSize < 1 || SADWindowSize % 2 != 1) {
#         printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
#     }
# 
#     Size img_size = img1.size();
# 
#     Mat Q;
#     Mat R, T, R1, P1, R2, P2;
# 
#     Mat map11, map12, map21, map22;
#     initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
#     initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
# 
#     Mat img1r, img2r;
#     remap(img1, img1r, map11, map12, INTER_LINEAR);
#     remap(img2, img2r, map21, map22, INTER_LINEAR);
# 
#     img1 = img1r;
#     img2 = img2r;
# 
#     numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
# 
#     sgbm->setPreFilterCap(63);
#     int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
#     sgbm->setBlockSize(sgbmWinSize);
# 
#     int cn = img1.channels();
# 
#     sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
#     sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
#     sgbm->setMinDisparity(0);
#     sgbm->setNumDisparities(numberOfDisparities);
#     sgbm->setUniquenessRatio(10);
#     sgbm->setSpeckleWindowSize(100);
#     sgbm->setSpeckleRange(32);
#     sgbm->setDisp12MaxDiff(1);
#     if(alg==STEREO_HH)
#         sgbm->setMode(StereoSGBM::MODE_HH);
#     else if(alg==STEREO_SGBM)
#         sgbm->setMode(StereoSGBM::MODE_SGBM);
# 
#     Mat disp, disp8;
# 
#     int64 t = getTickCount();
#     sgbm->compute(img1, img2, disp);
#     t = getTickCount() - t;
#     printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
# 
#     //disp = dispp.colRange(numberOfDisparities, img1p.cols);
#     if( alg != STEREO_VAR )
#         disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
#     else
#         disp.convertTo(disp8, CV_8U);
# 
#     namedWindow("left", 1);
#     imshow("left", img1);
#     namedWindow("right", 1);
#     imshow("right", img2);
#     namedWindow("disparity", 0);
#     imshow("disparity", disp8);
#     printf("press any key to continue...");
#     fflush(stdout);
#     waitKey();
#     printf("\n");
