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

void cameraMatrices(const cv::Mat & E, cv::Mat& M1, std::vector<cv::Mat>& M2s)
{   
    cout << "camera matrices" << endl;
    M1 = (Mat_<double>(3,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Mat S, U, V, result;
    SVD::compute(E, S, U, V);
    double m = (S.at<double>(0)+S.at<double>(1))/2;
    Mat temp = (Mat_<double>(3,3) << m, 0, 0, 0, m, 0, 0, 0, 0);
    Mat Ep = U * temp * V; // <------------- GOOD
    SVD::compute(Ep, S, U, V);
    Mat W = (Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);

    if (determinant(U*W*V)<0)
    {
        W = W*(-1);
    }

    cout << "test" << endl;

    temp = Mat(3, 3, CV_64F, 0);
    temp = U*W*V;
    double min, max;
    cv::minMaxLoc(abs(U.col(2)), &min, &max);
    Mat calc = U.col(2)/max;
    hconcat(temp.clone(), calc, result);
    M2s.push_back(result.clone());

    hconcat(temp.clone(), calc * -1, result);
    M2s.push_back(result.clone());

    temp = U*W.t()*V;
    hconcat(temp.clone(), calc, result);
    M2s.push_back(result.clone());

    hconcat(temp.clone(), calc * -1, result);
    M2s.push_back(result.clone());


        // for(unsigned int i = 0; i < M2s.size(); ++i)
        // {
        //     cout << M2s[i] << endl;
        // }
}

Mat skewSymmetricMatrix(const Point2f& p)
{
    return Mat_<double>(3,3) << 0, 1, -p.y, -1, 0, p.x, p.y, -p.x, 0;
}

void triangulate(const Mat & M1, const Mat & M2, vector<Point2f>& left, vector<Point2f>& right, std::vector<Point3f>& Ps)
{
    cout << "\t ----- new M2" << endl;
    Mat skmLeft, skmRight, result, S, U, V, temp, point;
    for(unsigned int i = 0; i < left.size(); ++i)
    {
        skmLeft = skewSymmetricMatrix(left[i]);
        skmRight = skewSymmetricMatrix(right[i]);

        vconcat(skmLeft * M1, skmRight * M2, result);

        SVD::compute(result, S, U, V);
        V = V.t();
        temp = V.col(V.cols - 1);
        point = temp/temp.at<double>(3);
        //cout << point << endl;
        Ps.push_back(Point3f(point.at<double>(0), point.at<double>(1), point.at<double>(2)));
    }
}

void calibratedEpipolarGeometry(cv::Mat& F, Mat& leftImage, Mat& rightImage, vector<Point2f>& left, vector<Point2f>& right)
{
    cout << "Prout" << endl;
    cv::Mat M1, skm;
    vector<Mat> M2s;
    
    float alpha = f * ppcm;
    Size leftSize = leftImage.size();
    float u = leftSize.width / 2;
    float v = leftSize.height / 2;
    double data[9] = {alpha,0,u,0,alpha,v,0,0,1};
    //double data[9] = {1520.4,0,302.32,0,1525.9,246.87,0,0,1};
    Mat K = Mat(3, 3, CV_64F, &data);
    cout << K << endl;
    Mat E = K.t() * F * K;

    cameraMatrices(E, M1, M2s);
    std::vector<Point3f> Pfinal;
    int nbNegMin = left.size(), nbNeg;

    for(unsigned int i = 0; i < M2s.size(); ++i)
    {
        nbNeg = 0;
        std::vector<Point3f> Ps;
        triangulate(K * M1, K * M2s[i], left, right, Ps);
        for(unsigned int j = 0; j < Ps.size(); ++j)
        {
            if(Ps[j].z <= 0)
            {
                ++nbNeg;
            }
        }
        if(nbNeg < nbNegMin) // We accept some percent of errors
        {
            cout << "Best M2 is : " << i << " with " << nbNeg << endl;
            Pfinal = Ps;
            nbNegMin = nbNeg;
        }
    }

    cout << "Reprojection and saving" << endl;
    ofstream myfile;
    myfile.open ("example.csv");
    for(unsigned int i = 0; i < Pfinal.size(); ++i)
    {
        if(Pfinal[i].z >= 0)
        {
            myfile << Pfinal[i].x << "," << Pfinal[i].y << "," << Pfinal[i].z << "\n";
            Mat pointFinal = (Mat_<double>(4,1) << Pfinal[i].x, Pfinal[i].y, Pfinal[i].z, 1);
            cout << "\t" << left[i] << endl << "\t" << K * M1 * pointFinal << endl << "\t-----" << endl;
        }
    }
    myfile.close();
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