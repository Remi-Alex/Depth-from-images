// g++ -c *.cpp; g++ $(pkg-config --cflags --libs opencv) main.cpp display.o geometry.o depth.o -o Test 

// Project
#include "display.h"
#include "geometry.h"
#include "depth.h"

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Native
#include <iostream>
#include <stdlib.h>     /* abs */
#include <stdio.h>
#include <vector>
#include <limits>       // std::numeric_limits
#include <sstream>
 
using namespace std;
using namespace cv;

Mat F;

vector<Point2f> calibratedPoints;

int main( int argc, const char** argv )
{
    /* ----- IMAGE OPENING AND COLOR CONVERSION ----- BEGIN */
    Mat leftImage, rightImage;
    Mat originalLeftImage = imread("../images/kinectL.bmp"); // query
    Mat originalRightImage = imread("../images/kinectR.bmp"); // train    

    cvtColor(originalLeftImage, leftImage, CV_BGR2GRAY);
    cvtColor(originalRightImage, rightImage, CV_BGR2GRAY);
    /* ----- IMAGE OPENING AND COLOR CONVERSION ----- END */
    
    /* ----- KEYPOINT DETECTION AND MATCHING ----- BEGIN */
    vector< Point2f > left;
    vector< Point2f > right;
    correspondences(leftImage, rightImage, left, right);
    manualCorrespondencesEpipolar(left, right);
    drawPoints(originalLeftImage, originalRightImage, left, right);
    /* ----- KEYPOINT DETECTION AND MATCHING ----- END */

    // Calculate fundamental matrix
    F = findFundamentalMat(Mat(left),Mat(right),CV_FM_LMEDS);

    // Find epilines
    vector< Vec3f > lines_left, lines_right;
    computeCorrespondEpilines(right, 2, F, lines_left);
    drawEpilines(originalLeftImage, lines_left, left);
    computeCorrespondEpilines(left, 1, F, lines_right);
    drawEpilines(originalRightImage, lines_right, right);

    imshow("Epipolar lines right", originalLeftImage);
    imshow("Epipolar lines left", originalRightImage);

    calibratedEpipolarGeometry(F, leftImage, rightImage, left, right);

    imshow("resultat trop boooo", leftImage);
    waitKey(0);

    return 0;
}
