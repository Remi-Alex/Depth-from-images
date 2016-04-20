#ifndef DFI_GEOMETRY
#define DFI_GEOMETRY

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <vector>

#define MIN_HESSIAN 800

bool lineIntersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2, cv::Point2f &r);

bool epilineIntersection(cv::Mat& img, std::vector<cv::Vec3f>& lines, cv::Point2f& intersection);

void correspondences(cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void manualCorrespondencesEpipolar(std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

void manualCorrespondencesDisparity(std::vector<cv::Point2f>& calibratedPoints);

#endif  // DFI_GEOMETRY
