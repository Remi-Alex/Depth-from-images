#ifndef DFI_DISPLAY
#define DFI_DISPLAY

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>

void drawEpilines(cv::Mat & img, std::vector<cv::Vec3f> lines, std::vector<cv::Point2f> points);

void drawPoints(cv::Mat & left_img, cv::Mat & right_img, std::vector<cv::Point2f> & left, std::vector<cv::Point2f> & right);

#endif  // DFI_DISPLAY
