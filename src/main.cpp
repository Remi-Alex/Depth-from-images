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

void drawPoints(Mat & left_img, Mat & right_img, vector<Point2f> & left, vector<Point2f> & right)
{
	RNG rng(12345);
	for (unsigned int i = 0; i < left.size(); ++i)
	{
		Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		circle(left_img, left[i], 5, color);
		circle(right_img, right[i], 5, color);
	}
}

int main( int argc, const char** argv )
{
    /* ----- AUTO CALIBRATION ----- BEGIN */
    Mat originalLeftImage = imread("images/left.jpg"); // query
    Mat originalRightImage = imread("images/right.jpg"); // train

    Mat leftImage, rightImage;

    cvtColor(originalLeftImage, leftImage, CV_BGR2GRAY);
    cvtColor(originalRightImage, rightImage, CV_BGR2GRAY);

    // Keypoints
    SurfFeatureDetector detector(MIN_HESSIAN);
    std::vector< KeyPoint > kp_left, kp_right;
    detector.detect(leftImage, kp_left);
    detector.detect(rightImage, kp_right);

    // Descriptors
    SurfDescriptorExtractor extractor;
    Mat desc_left, desc_right;
    extractor.compute( leftImage, kp_left, desc_left );
    extractor.compute( rightImage, kp_right, desc_right );

    // FLANN matches
    FlannBasedMatcher matcher;
    //BFMatcher matcher(NORM_L1, true);
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

    for(unsigned int i = 0; i < matches.size(); ++i)
	{
    	float min = matches[i].distance;
    	unsigned int min_index = i;
		for(unsigned int j = i + 1; j < matches.size(); ++j)
		{
			if(matches[j].distance < min)
			{
				min = matches[j].distance;
				min_index = j;
			}
		}
		DMatch temp = matches[i];
		matches[i] = matches[min_index];
		matches[min_index] = temp;
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

//    drawPoints(originalLeftImage, originalRightImage, left, right);
//    imshow("left", originalLeftImage);
//    imshow("right", originalRightImage);
//    waitKey(0);

    // Calculate fundamental matrix
    Mat mask;
    Mat F = findFundamentalMat(Mat(left),Mat(right),CV_FM_LMEDS, 3., 0.99, mask);



    // Find epilines
    vector< Vec3f > lines_left, lines_right;
    computeCorrespondEpilines(right, 2, F, lines_left);
    drawEpilines(originalLeftImage, lines_left, left);
    computeCorrespondEpilines(left, 1, F, lines_right);
    drawEpilines(originalRightImage, lines_right, right);

//    Size leftSize = leftImage.size();
//	Size newSize(leftSize.width/5, leftSize.height/5);
//	resize(leftImage, leftImage, newSize);
//	resize(rightImage, rightImage,newSize);

    imshow("left", originalLeftImage);
    imshow("right", originalRightImage);
    imwrite("leftResult.png", originalLeftImage);
    imwrite("rightResult.png", originalRightImage);
    waitKey(0);

    // ***************DISPARITY************************
//    	Mat disparity;
//    	StereoBM stereo = StereoBM();
//    	stereo.init(1,16,15);
//    	stereo(leftImage, rightImage, disparity);
//
//    	disparity.convertTo(disparity, CV_8UC1);
////		Size size = disparity.size();
////		Size newSize(size.width/5, size.height/5);
////		resize(disparity, disparity, newSize);
//    	imshow("Disparity map", disparity);
//    	waitKey(0);
    //*************************************************


    Point3f Ol(0,0,0);
    float f = 366; //in pixels because fuck you that's why
    float rotationY = 45; //in degrees motherfucka
    float translationX = 30; //in cm BITCH
    float translationZ = 12.5; //in cm also crazy whore

    Size leftSize = leftImage.size();
    Point3f OimgL(-leftSize.width/2, leftSize.height/2, f);


    return 0;
}
