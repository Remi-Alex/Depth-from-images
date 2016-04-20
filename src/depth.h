#ifndef DFI_DEPTH
#define DFI_DEPTH

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <fstream>
#include <vector>

void calibratedEpipolarGeometry(cv::Mat& F, cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void betterDisparity(cv::Mat& F, cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right, std::vector<cv::Point2f>& calibratedPoints);

void disparityMethod(cv::Mat& leftImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void disparityMethodCalibrated(cv::Mat& leftImage, std::vector<cv::Point2f>& calibratedPoints);

void cameraMatrices(const cv::Mat & E, cv::Mat& M1, std::vector<cv::Mat>& M2s);

void triangulate(const cv::Mat & M1, const cv::Mat & M2, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right, std::vector<cv::Point3f>& Ps);

cv::Mat skewSymmetricMatrix(const cv::Point2f& p);

#endif  // DFI_DEPTH
