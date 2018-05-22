#pragma once
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include <string>
#include <random>
#include <iomanip>
#include <arm_neon.h>
#include <omp.h>
#include "codes.hpp"
#include "helper.hpp"

using namespace cv;

/**
 * global coord system
 * 
 * 	l/l	    x    l/h 
 *     -|--------> 
 *    y |
 *      |
 * h/l  v        h/h
 *
 * h: high, l: low
 */

/**
 * Conducts Acitve Line Modelling (ALM) on the input points (belonging pariwise to lines from a Hough transform after get_points())
 * @param img Input image
 * @param left_points Holds the found points of the left lane
 * @param right_points Holds the found points of the right
 * @param num_part Number of partitions per side
 * @param num_lines Number of lines per partition
 * @param b_view Bird View on or off
 * @param roi Start point of ROI in percent (y-axis)
 * @return Returns either LANEDET_WARNING or LANEDET_SUCESS
 * @note calls alm(std::vector<Point2f> &, const int, const int) once for each side
 */
int alm(const Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part,
        const int num_lines, const bool b_view, const double roi);

/**
 * Conducts a simple Hough line search in multiple partitions (in each only one line per side is found)
 * @param img Input image
 * @param left_points Holds the found points of the left lane
 * @param right_points Holds the found points of the right
 * @param num_part Number of partitions per side
 * @param b_view Bird View on or off
 * @param roi Start point of ROI in percent (y-axis)
 * @return Returns either LANEDET_WARNING or LANEDET_SUCESS
 */
int hough(Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points,
          const int num_part, const bool b_view, const double roi);


/**
 * Performs random line search 
 * @param img Image to be searched for road lanes
 * @param num_lines Number of random lines per partition per side used to detected lanes
 * @param roi Region of interest
 * @param num_part Number of partitions 
 * @param b_view Indicator if we need to adjust some parameters, if bird-view is used (not used right now)
 * @param left_points Vector returning detected points of left lane ready for fitting
 * @param right_points Vector returning detected points of right lane ready for fitting
 */
int random_search(Mat &img, const int num_lines, const double roi, const int num_part, const bool b_view,
        std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * Takes the histogram of the upper half of the image as a startig point for 
 * a sliding windows search along the road lanes (UDACITY Advance Lane course).
 * @param img Image to be searched for road lanes
 * @param roi Vertical starting point in percent of the region of interest (for the h_histrogram() call within)
 * @param num_windows The number of windows for both lanes (left and right)
 * @param width The width (in x-direction) of each windows
 * @param left_points The returned points which were found in the window search on the left side 
 * @param right_points The returned points which were found in the window search on the right side 
 * @return Returns either LANEDET_SUCCESS, LANEDET_WARNING
 * @note Calls sliding_window_search(Mat &, const double, const int, const int, std::vector<Point2f> &, const bool) once for each side
 */
int sliding_windows_search(Mat &img, const double roi, const int num_windows, const int width,
                           std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * Given two input-window-starting-points (from a h_histogram), search those in three different places along the y-axis 
 * to find 6 total points of the two curved road lane
 * @param img image to be searched for the six road lane points
 * @param window_width width of one of the two search windows
 * @param roi Percent of the image rows where the region of interest start
 * @param output_points returns the six points found in the image. If no suitable point is found (-1,-1) is returned
 * @note can be prallelized for the six independent regions -> early aborting as soon as a point is found
 * @note points are saved from top (high y values) to bottom (low y values) in [left/right]_points
 * @return Returns either LANEDET_SUCCESS, LANEDET_WARNING
 */
int fixed_window_search(const Mat &img, const int window_width, const double roi,
                  std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);
