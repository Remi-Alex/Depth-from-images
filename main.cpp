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
 
void drawEpilines(Mat & img, vector< Vec3f> lines, vector< Point2f > points)
{
    for (std::vector<Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
    {
        cv::line(
            img,
            Point(0,-(*it)[2]/(*it)[1]),
            Point(img.cols,-((*it)[2]+(*it)[0]*img.cols)/(*it)[1]),
            Scalar(255,255,255)
        );
    }

    for (std::vector<Point2f>::const_iterator pt = points.begin(); pt!=points.end(); ++pt)
    {
        circle(img, *pt, 5, Scalar(255,0,0));
    }
}

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

    // Select good matches
    vector< DMatch > good_matches;
    vector< Point2f > left;
    vector< Point2f > right;
    double max_dist = 0; double min_dist = 100;
    DMatch current_match;

    for( int i = 0; i < desc_left.rows; i++ ) // Quick calculation of max and min distances between keypoints
    { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    for( int i = 0; i < desc_left.rows; i++ )
    { 
        if( matches[i].distance < 3*min_dist )
        {
            current_match = matches[i];
            good_matches.push_back( current_match );
            left.push_back( kp_left[ current_match.queryIdx ].pt );
            right.push_back( kp_right[ current_match.trainIdx ].pt );
        }
    }

    // Calculate fundamental matrix
    Mat F = findFundamentalMat(Mat(left),Mat(right),CV_FM_LMEDS);

    // Find epilines 
    vector< Vec3f > lines_left, lines_right;
    computeCorrespondEpilines(right, 2, F, lines_left);
    drawEpilines(leftImage, lines_left, left);
    computeCorrespondEpilines(left, 1, F, lines_right);
    drawEpilines(rightImage, lines_right, right);
    
    imshow("left", leftImage);
    imshow("right", rightImage);
    waitKey(0);

    return 0;
}