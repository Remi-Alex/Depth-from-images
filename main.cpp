// g++ $(pkg-config --cflags --libs opencv) main.cpp -o Test 

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
#include <iostream>
#include <stdio.h>
#include <vector>
 
using namespace std;
using namespace cv;

/* ----- DEFINES ----- BEGIN */
#define MIN_HESSIAN 800
/* ----- DEFINES ----- END */
 
int main( int argc, const char** argv )
{
    /* ----- AUTO CALIBRATION ----- BEGIN */
    Mat leftImage = imread("images/left.jpg"); // query
    Mat rightImage = imread("images/right.jpg"); // train

    // Keypoints
    SiftFeatureDetector detector(MIN_HESSIAN);
    std::vector< KeyPoint > kp_left, kp_right;
    detector.detect(leftImage, kp_left);
    detector.detect(rightImage, kp_right);
    
    // Descriptors
    SiftDescriptorExtractor extractor;
    Mat desc_left, desc_right;
    extractor.compute( leftImage, kp_left, desc_left );
    extractor.compute( rightImage, kp_right, desc_right );

    // FLANN matches
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( desc_left, desc_right, matches );

    return 0;
}