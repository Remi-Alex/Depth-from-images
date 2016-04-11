#ifndef DFI_DEPTH
#define DFI_DEPTH

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <vector>

void calibratedEpipolarGeometry(cv::Mat& F, cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void betterDisparity(cv::Mat& F, cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right, std::vector<cv::Point2f>& calibratedPoints);

void disparityMethod(cv::Mat& leftImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void disparityMethodCalibrated(cv::Mat& leftImage, std::vector<cv::Point2f>& calibratedPoints);

#endif  // DFI_DEPTH
