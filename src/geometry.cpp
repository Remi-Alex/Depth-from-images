#include "geometry.h"

using namespace std;
using namespace cv;

bool lineIntersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
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

void correspondences(Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right)
{
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
}

void manualCorrespondencesEpipolar(vector<Point2f>& left, vector<Point2f>& right)
{
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
}

void manualCorrespondencesDisparity(vector<Point2f>& calibratedPoints)
{
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
}
