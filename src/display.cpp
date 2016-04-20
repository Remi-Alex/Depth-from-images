#include "display.h"

using namespace std;
using namespace cv;

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

    //for (std::vector<Point2f>::const_iterator pt = points.begin(); pt!=points.end(); ++pt)
    //{
    //    circle(img, *pt, 5, Scalar(255,0,0));
    //}
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
