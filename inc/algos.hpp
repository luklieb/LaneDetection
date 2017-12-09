#pragma once
#include <opencv/cv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/cv_cpu_helper.h>

using namespace cv;

void
HoughLinesCustom(const Mat &img, float rho, float theta,
                   int threshold, std::vector<Vec2f> &lines, int linesMax,
                   double min_theta, double max_theta);


