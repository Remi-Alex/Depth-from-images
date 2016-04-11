#include "depth.h"

using namespace std;
using namespace cv;

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

void calibratedEpipolarGeometry(cv::Mat& F, Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right)
{
    float alpha = f * ppcm;
    Size leftSize = leftImage.size();
    float u = leftSize.width / 2;
    float v = leftSize.height / 2;
    double data[9] = {alpha,0,u,0,alpha,v,0,0,1};
    Mat K = Mat(3, 3, CV_64F, &data);
    Mat E = K.t() * F * K;

    Mat M1 = (Mat_<double>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Mat C1 = K * M1;

    cout << "C1 : " << C1 << endl;

    SVD svd(E,SVD::MODIFY_A);
    Mat svd_u = svd.u;
    Mat svd_vt = svd.vt;
    Mat svd_w = svd.w;
    Matx33d W(0,-1,0,1,0,0,0,0,1);//HZ 9.13
    Mat_<double> R = svd_u * Mat(W) * svd_vt; //
    Mat_<double> T = svd_u.col(2); //u3

    Mat M2 = Mat_<double>(3,4);
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            M2.at<double>(i,j) = R.at<double>(i,j);
        }
        M2.at<double>(i, 3) = T.at<double>(i);
    }

    Mat C2 = K * M2;

    cout << "C2 : " << C2 << endl;

    vector<Point3f> pLeft, pRight;
    Mat A(4,4,CV_64F, double(0));
    for(unsigned int i = 0; i < left.size(); ++i)
    {
        A.row(0) = left[i].x * C1.row(2) - C1.row(0);
        A.row(1) = left[i].y * C1.row(2) - C1.row(1);
        A.row(2) = right[i].x * C2.row(2) - C2.row(0);
        A.row(3) = right[i].y * C2.row(2) - C2.row(1);

        cout << "left : " << left[i] << "\nright : " << right[i] << endl;

        SVD svd2(A);
        Mat result = svd2.w/svd2.w.at<double>(3);

        cout << "result : " << result << endl;
        Mat reproj = C1 * result;
        cout << "left reproject : " << reproj/reproj.at<double>(2) << endl;

        pLeft.push_back(Point3f(left[i].x, left[i].y, 1));
        pRight.push_back(Point3f(right[i].x, right[i].y, 1));
    }
}

void betterDisparity(cv::Mat& F, Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right, std::vector<cv::Point2f>& calibratedPoints)
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

void disparityMethodCalibrated(Mat& leftImage, std::vector<cv::Point2f>& calibratedPoints)
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