#pragma once
#include <opencv/cv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/cv_cpu_helper.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <numeric>
#include <cmath>

using namespace cv;

void HoughLinesCustom(const Mat &img, float rho, float theta,
                   int threshold, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines, 
                   int linesMax, int roi_start, int roi_end, double min_theta=0, double max_theta=CV_PI);

void v_roi(Mat &img, const int &start);

//Deprecated
void bird_view(const Mat &input_img, Mat &output_img, double rel_height, double rel_left, double rel_right);

void h_histogram(const Mat &input_img, int *points);

void window_search(const Mat &img, const int *input_points, const int &window_width, 
                    std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

void multiple_windows_search(Mat &input_img, const int num_windows, const int width, 
                    std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

void draw_curve(Mat &image, const std::vector<Point> &points);

void poly_reg(const std::vector<Point> &points, std::vector<double> &coeff);


