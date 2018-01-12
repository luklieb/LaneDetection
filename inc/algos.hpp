#pragma once
#include <opencv/cv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/cv_cpu_helper.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <numeric>

using namespace cv;

void HoughLinesCustom(const Mat &img, float rho, float theta,
                   int threshold, std::vector<Vec2f> &lines, int linesMax,
                   int roi_start, int roi_end, double min_theta=0, double max_theta=CV_PI);

void v_roi(Mat &img, const int &start);

void bird_view(const Mat &input_img, Mat &output_img, double rel_height, double rel_left, double rel_right);

void h_histogram(const Mat &input_img, int *points);

void window_search(const Mat &img, const int *input_points, const int &window_width, 
                    std::vector<Point> &output_points);

void draw_curve(Mat &image, const std::vector<Point> points, const uint start);


