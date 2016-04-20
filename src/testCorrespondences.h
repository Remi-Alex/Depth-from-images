#ifndef DFI_TEST_CORR
#define DFI_TEST_CORR

#include "opencv2/core/core.hpp"

#include <iostream>
#include <vector>

void testManualCorrespondencesEpipolar(std::vector<cv::Point2f>& left, std::vector<cv::Point2f>& right);

#endif  // DFI_TEST_CORR