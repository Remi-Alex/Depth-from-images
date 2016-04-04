// g++ $(pkg-config --cflags --libs opencv) main.cpp -o Test 

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
#include <iostream>
#include <stdlib.h>     /* abs */
#include <stdio.h>
#include <vector>
#include <limits>       // std::numeric_limits
#include <sstream>
 
using namespace std;
using namespace cv;

/* ----- DEFINES ----- BEGIN */
#define MIN_HESSIAN 430 // OTHER
//#define MIN_HESSIAN 300 // epipo 
/* ----- DEFINES ----- END */

Point3f Ol(0,0,0);
Point3f OimgL(0,0,0);
Point3f OimgR(0,0,0);
float f = 53.19; //in cm !!!!!!!
float rotationY = -45; //in degrees motherfucka
float translationX = 30; //in cm BITCH
float translationZ = 12.5; //in cm also crazy whore
float ppcm = 20.11; //pixels per cm
float cmpp = 0.0497; //cm per pixel
float T = 25; //cm (distance between two cameras
Point3f Or(translationX, 0, translationZ);
float rotationConstX = Or.x*(1-cos(rotationY)) + Or.z * sin(rotationY);
float rotationConstZ = Or.y*(1-cos(rotationY)) - Or.x * sin(rotationY);
Mat F;


vector<Point2f> calibratedPoints;

bool lineIntersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                      Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
    {
        cout << abs(cross) << endl;
        return false;
    }

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

bool epilineIntersection(Mat& img, vector< Vec3f >& lines, Point2f& intersection)
{
    Point l1a(0,-lines[0][2]/lines[0][1]);
    Point l1b(img.cols,-(lines[0][2]+lines[0][0]*img.cols)/lines[0][1]);

    Point l2a(0,-lines[3][2]/lines[3][1]);
    Point l2b(img.cols,-(lines[3][2]+lines[3][0]*img.cols)/lines[3][1]);

    return lineIntersection(l1a, l1b, l2a, l2b, intersection);
}

Point3f get3DPoint(Point3f xl, Point3f xr)
{
    Point3f leftDir = xl - Ol;
    leftDir.x /= norm(leftDir);
    leftDir.y /= norm(leftDir);
    leftDir.z /= norm(leftDir);
    Point3f rightDir = xr - Or;
    rightDir.x /= norm(rightDir);
    rightDir.y /= norm(rightDir);
    rightDir.z /= norm(rightDir);
    Point3f tempPoint;

    Point3f maxR = xl + 300 * rightDir;

    float distance = std::numeric_limits<float>::max(), distTemp;
    int goodT = -1;
    for(int t = 0; t < 300; ++t)
    {
    	tempPoint = xl + t * leftDir;
    	distTemp = norm((tempPoint - xr).cross(tempPoint-maxR))/norm(maxR-xr);
    	if(distTemp < distance)
    	{
    		goodT = t;
    		distance = distTemp;
    	}
    }

    return Point3f(xl + goodT * leftDir);
}

Point3f applyRotationY()
{
	Point3f result(0,Or.y,0);
	result.x = Or.x * cos(rotationY) - Or.z * sin(rotationY) + rotationConstX;
	result.z = Or.x * sin(rotationY) + Or.z * cos(rotationY) + rotationConstZ;
	return result;
}

Point3f getIn3D(Point2f point, bool left)
{
	Point3f result(0,0,0);

	if(left)
	{
		result = Point3f(OimgL.x + point.x*cmpp, OimgL.y - point.y*cmpp, f);
	}
	else
	{
		result = Point3f(OimgR.x + point.x*cmpp, OimgR.y - point.y*cmpp, f);
		result = applyRotationY();
	}

	return result;
}

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

void epipolarGeometry(Mat& leftImage, vector<Point2f>& left, vector<Point2f>& right)
{
    Size leftSize = leftImage.size();
    OimgL = Point3f((-leftSize.width/2)*cmpp, (leftSize.height/2)*cmpp, f);

    OimgR = OimgL + Or; //translation
    OimgR = applyRotationY();

    for(unsigned int l = 0; l < 5; ++l)
    {
    	Point2f pLeft = left[l];
    	Point2f pRight = right[l];

    	cout << getIn3D(pLeft, true) << endl;

    	Point3f res = get3DPoint(getIn3D(pLeft, true), getIn3D(pRight, false));

    	cout << res << endl;
    	cout << "*********" << endl;
    }
}

void calibratedEpipolarGeometry(Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right, Point3f& ePrime)
{
    // float alpha = f * ppcm;
    // Size leftSize = leftImage.size();
    // float u = leftSize.width / 2;
    // float v = leftSize.height / 2;
    // float data[9] = {alpha,0,u,0,alpha,v,0,0,1};
    // Mat K = Mat(3, 3, CV_64F, &data);
    // Mat E = K.t() * F * K;

    // cout << E << endl;

    // SVD svd(E,SVD::MODIFY_A);
    // Mat svd_u = svd.u;
    // Mat svd_vt = svd.vt;
    // Mat svd_w = svd.w;
    // Matx33d W(0,-1,0,1,0,0,0,0,1);//HZ 9.13
    // Mat_<double> R = svd_u * Mat(W) * svd_vt; //
    // Mat_<double> T = svd_u.col(2); //u3

    // cout << R << " " << T << endl;

    double pData[12] = {1,0,0,0,0,1,0,0,0,0,1,0};
    Mat P(3, 4, CV_64F, &pData);

    cout << "P :      " << P << endl;

    double sData[9] = {0,-ePrime.z,ePrime.y,ePrime.z,0,-ePrime.x,-ePrime.y,ePrime.x,0};
    Mat S(3,3,CV_64F,&sData);

    cout << ePrime << endl;

    cout << "F :      " << F << endl;

    cout << "S :      " << S << endl; 

    Mat inter = S * F;

    cout << "inter :      " << inter << endl;

    Mat pPrime(3, 4, CV_64F);
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            pPrime.at<double>(i,j)=inter.at<double>(i,j);
        }
    }
    pPrime.at<double>(0,3)=ePrime.x;
    pPrime.at<double>(1,3)=ePrime.y;
    pPrime.at<double>(2,3)=ePrime.z;

    cout << "pPrime :      " << pPrime << endl;

    Mat A(4,4,CV_64F, double(0));
    Mat B = Mat(4,1, CV_64F, double(0));
    Mat x = Mat(4,1, CV_64F);
    for(unsigned int i = 0; i < left.size(); ++i)
    {
        A.row(0) = left[i].x * P.row(2) - P.row(0);
        A.row(1) = left[i].y * P.row(2) - P.row(1);
        A.row(2) = right[i].x * pPrime.row(2) - pPrime.row(0);
        A.row(3) = right[i].y * pPrime.row(2) - pPrime.row(1);

        B = A.col(3).clone() ;

        for(int j = 0; j < 4; ++j)
        {
            A.at<double>(j, 3) = 0;
        }

        cout << "A :      " << A << endl;
        cout << "B :      " << B << endl;

        solve(A, B, x);
        cout << "x :      " << A.inv() * B << endl;
        cout << "sol :    " << x/x.at<double>(3) << endl;
    }
}

void betterDisparity(Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right)
{
    cv::Mat H1(4,4, leftImage.type());
    cv::Mat H2(4,4, leftImage.type());
    cv::stereoRectifyUncalibrated(left, right, F, leftImage.size(), H1, H2);

    // create the image in which we will save our disparities
    Mat imgDisparity16S = Mat( leftImage.rows, leftImage.cols, CV_16S );
    Mat imgDisparity8U = Mat( leftImage.rows, leftImage.cols, CV_8UC1 );

    // Call the constructor for StereoBM
    int ndisparities = 16;      // < Range of disparity >
    int SADWindowSize = 5;        // < Size of the block window > Must be odd. Is the 
                                  // size of averaging window used to match pixel  
                                  // blocks(larger values mean better robustness to
                                  // noise, but yield blurry disparity maps)

    StereoBM sbm( StereoBM::BASIC_PRESET,
        ndisparities,
        SADWindowSize );

    // Calculate the disparity image
    sbm( leftImage, rightImage, imgDisparity16S, CV_16S );

    // Check its extreme values
    double minVal; double maxVal;

    minMaxLoc( imgDisparity16S, &minVal, &maxVal );

    printf("Min disp: %f Max value: %f \n", minVal, maxVal);

    // Display it as a CV_8UC1 image
    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

    namedWindow( "windowDisparity", CV_WINDOW_NORMAL );
    imshow( "windowDisparity", imgDisparity8U );

    cout << imgDisparity16S.at<short>(calibratedPoints[0].x, calibratedPoints[0].y) << endl;
}


void disparityMethod(Mat& leftImage, vector<Point2f>& left, vector<Point2f>& right)
{
	for(unsigned int l = 0; l < left.size(); ++l)
	{
		Point2f pLeft = left[l];
		Point2f pRight = right[l];
		float dist = T/((pLeft.x*cmpp) - (pRight.x*cmpp)) * f;
		circle(leftImage, left[l], 5, Scalar(255,0,0));
		ostringstream buff;
		buff << dist;
		String s = buff.str();
		putText(leftImage, s, Point2f(left[l].x + 8, left[l].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,0));

	}
}

void disparityMethodCalibrated(Mat& leftImage)
{
	for(unsigned int l = 0; l < calibratedPoints.size(); l+=2)
	{
		Point2f pLeft = calibratedPoints[l];
		Point2f pRight = calibratedPoints[l+1];
		float dist = T/((pLeft.x*cmpp) - (pRight.x*cmpp)) * f;
		circle(leftImage, calibratedPoints[l], 10, Scalar(255,0,0), 3);
		ostringstream buff;
		buff << dist;
		String s = buff.str();
		putText(leftImage, s, Point2f(calibratedPoints[l].x + 8, calibratedPoints[l].y), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 3);

	}
}

int main( int argc, const char** argv )
{
    /* ----- AUTO CALIBRATION ----- BEGIN */
    Mat originalLeftImage = imread("../images/kinectL.bmp"); // query
    Mat originalRightImage = imread("../images/kinectR.bmp"); // train
    // Mat originalLeftImage = imread("../images/etalonnage/KinectScreenshot-Color-03-40-32.bmp"); // query
    // Mat originalRightImage = imread("../images/etalonnage/KinectScreenshot-Color-03-45-44.bmp"); // train
//	Mat originalLeftImage = imread("images/etalonnage/KinectScreenshot-Color-04-08-11.bmp"); // query
//	Mat originalRightImage = imread("images/etalonnage/KinectScreenshot-Color-04-07-33.bmp"); // train

    Mat leftImage, rightImage;

    cvtColor(originalLeftImage, leftImage, CV_BGR2GRAY);
    cvtColor(originalRightImage, rightImage, CV_BGR2GRAY);

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

    // ************************ HACK
    left.clear();
    right.clear();
    //clavier
    left.push_back(Point2f(550,47));
    right.push_back(Point2f(1175,34));

    left.push_back(Point2f(785,614));
    right.push_back(Point2f(837,587));

    left.push_back(Point2f(1227,470));
    right.push_back(Point2f(1484,404));

    left.push_back(Point2f(190,75));
    right.push_back(Point2f(880,153));

    left.push_back(Point2f(605,180));
    right.push_back(Point2f(1237,151));

    left.push_back(Point2f(882,368));
    right.push_back(Point2f(961,322));

    left.push_back(Point2f(1271,559));
    right.push_back(Point2f(1610,543));

    left.push_back(Point2f(348,94));
    right.push_back(Point2f(1001,137));
    //////////////// ********** FIN HACK

    drawPoints(originalLeftImage, originalRightImage, left, right);
//    Size leftSize = originalLeftImage.size();
//  	Size newSize(leftSize.width/2, leftSize.height/2);
//  	resize(originalLeftImage, originalLeftImage, newSize);
//  	resize(originalRightImage, originalRightImage,newSize);
//    imshow("left", originalLeftImage);
//    imshow("right", originalRightImage);

//    waitKey(0);

    // Calculate fundamental matrix
    F = findFundamentalMat(Mat(left),Mat(right),CV_FM_LMEDS);


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

    Point2f ePrime;
    if(!epilineIntersection(leftImage, lines_right, ePrime))
    {
        cout << "No intersection!" << endl;
        exit(-1337);
    }
    Point3f ePrimeH(ePrime.x, ePrime.y, 1);
//    waitKey(0);

    // ***************DISPARITY************************
//    	Mat disparity;
//    	StereoBM stereo = StereoBM();
//    	stereo.init(1,16,15);
//    	stereo(leftImage, rightImage, disparity);
//
//    	disparity.convertTo(disparity, CV_8UC1);
//		Size size = disparity.size();
//		Size newSize(size.width/2, size.height/2);
//		resize(disparity, disparity, newSize);
//    	imshow("Disparity map", disparity);
//    	waitKey(0);
    //*************************************************

    //epipolarGeometry(leftImage, left, right);

    //disparityMethod(leftImage, left, right);

    //clavier
    calibratedPoints.push_back(Point2f(1208,384));
    calibratedPoints.push_back(Point2f(707, 380));
    //batman
    calibratedPoints.push_back(Point2f(1145, 67));
    calibratedPoints.push_back(Point2f(987, 63));
    //canapé
    calibratedPoints.push_back(Point2f(401, 417));
    calibratedPoints.push_back(Point2f(207, 414));
    //coussin
    calibratedPoints.push_back(Point2f(1643, 572));
    calibratedPoints.push_back(Point2f(1386, 571));

    calibratedEpipolarGeometry(leftImage, rightImage, left, right, ePrimeH);

    //betterDisparity(leftImage, rightImage, left, right);

    //disparityMethodCalibrated(leftImage);
//    Size size = leftImage.size();
//	Size newSize = Size(size.width/2, size.height/2);
//	resize(leftImage, leftImage, newSize);
    imshow("resultat trop boooo", leftImage);
    imwrite("distance.png", leftImage);
    waitKey(0);



    return 0;
}
