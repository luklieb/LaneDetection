#pragma once
#include <opencv/cv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/cv_cpu_helper.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include <string>
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
 * @param image_start Start point in pixel (y-axis)
 * @return Returns either MAPRA_WARNING or MAPRA_SUCESS
 */
int alm(const Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part, 
        const int num_lines, const bool b_view, const int image_start);


/**
 * Converts the polar coordiantes from a Hough Transform (lines) to points according to their start/end of each partition
 * @param lines The polar coordinates returned from a Hough Transform
 * @param num_lines The number of lines of each side of each partition
 * @param num_part The number of partitions
 * @param coords_part Coordinates of the partitions
 * @param points Holds the converted Points 
 * @return Returns either MAPRA_SUCCESS or MAPRA_WARNING
 * @note Storage order for both [left/right]_points: points belonging to same line are stored consecutively:
 *      Point_0(line_0), Point_1(line_0), Point_2(line_1), Point_3(line_1), ..., 
 *      Point_num_lines*2-1(line_num_lines-1), ..., Point_num_lines*num_part*2-1(line_num_lines*num_part-1) 
 */
int get_points(const std::vector<Vec2f> &left_lines, const std::vector<Vec2f> &right_lines, const int num_lines, const int num_part,
               const int *coords_part, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * Returns the two horizontal (x-coordinates) points (one for right/left half) with the maximum according to the histogram of the ROI
 * @param input_img is the binary input image
 * @param roi Vertical starting point in percent of the region of interest
 * @param points holds the 2 x-coordinates according to the two max values of the histogram
 * @return Returns either MAPRA_SUCCESS or MAPRA_WARNING
 * @note reduce() function could be optimized with an additional roi constraint
 */
int h_histogram(const Mat &input_img, const double roi, int *points);

/**
 * Conducts a simple Hough line search in multiple partitions (in each only one line per side is found)
 * @param img Input image
 * @param left_points Holds the found points of the left lane
 * @param right_points Holds the found points of the right
 * @param num_part Number of partitions per side
 * @param num_lines Number of lines per partition
 * @param b_view Bird View on or off
 * @param image_start Start point in pixel (y-axis)
 * @return Returns either MAPRA_WARNING or MAPRA_SUCESS
 */
int hough(const Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part,
        const int num_lines, const bool b_view, const int image_start);

/**
 * Returns polar coordinates (rho and theta) of the found lines
 * Copied from github opencv/modules/imgproc/src/hough.cpp, but adapted to improve lane detection performance
 * @param img Input image 
 * @param rho Distance resolution parameter in pixels
 * @param theta Angle resolution parameter in radiants
 * @param threshold Minimum number of intersections to detect a line
 * @param left_lines Vector returning the found lines_max lines (in rho and theta) of the left side
 * @param right_lines Vector returning the found lines_max lines (in rho and theta) of the right side
 * @param lines_max Number of lines to be found per side
 * @param roi_start Start of the horizontal region of interest in pixels (probably from sub_partition())
 * @param roi_end End of the horizontal region of interest in pixels (probably from sub_partition())
 * @param b_view If true, Hough Transformation is slightly optimized for the bird-view-perspective
 * @param min_theta Should be 0
 * @param max_theta Should be CV_PI
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING or MAPRA_ERROR
 * @note be aware that if the current line is similar (parrallel) to the previous line it gets ignored
 *       -> might cause problems with "bird eye view" (because then both road lines are parallel)
 * @note: add further failsafe (one more bool to check wheter birdsview or not and then
 * add restrictions in function to restrict e.g. the angle of the second line, etc.)
 */
int hough_lines_custom(const Mat &img, const float rho, const float theta,
                     const int threshold, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines,
                     const int linesMax, const int roi_start, const int roi_end, const bool b_view,
                     const double min_theta = 0, const double max_theta = CV_PI);

/**
 * Converts the points-pairs making up lines returned from alm() or partitoned_hough() with num_lines = 1
 * into single points (in order to compute the coefficients)
 * It computes the x-cooordinate average of two points with the same y-coordinate (two adjacent lines)
 * Before each line has its own start and end point, after the call each end point of a line segment 
 * is the start point of the next line segment
 * @param left_points Holds the points-pairs for the left lane
 * @param left_points Holds the points-pairs for the right lane
 * @return Returns either MAPRA_WARNING or MAPRA_SUCESS
 */
int pair_conversion(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * Applies the Hough Transformation on different sub-regions 
 * @param img The image for the Hough Transformation
 * @param part_coords Holds the num_part+1 coordinates of the partitions
 * @param num_part Number of sub-domains/partitions
 * @param num_lines Number of lines per partition per side 
 * @param left_lines Vector holding the polar coordinates of the detected lines on the left side (lane)
 * @param right_lines Vector holding the polar coordinates of the detected lines on the right side (lane)
 * @param b_view If true, Hough Transformation is slightly optimized for the bird-view-perspective
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING
 * @note [left/right]_lines stores at the beginning num_lines lines of the first partition,
 * 		at the end num_lines lines of the last partition
 */
int partitioned_hough(const Mat &img, const int *part_coords, const int num_part, const int num_lines,
                      std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines, const bool b_view);



/**
 * Takes the histogram of the upper half of the image as a startig point for 
 * a sliding windows search along the road lanes (UDACITY Advance Lane course).
 * @param input_img Image to be searched for road lanes
 * @param roi Vertical starting point in percent of the region of interest (for the h_histrogram() call within)
 * @param num_windows The number of windows for both lanes (left and right)
 * @param width The width (in x-direction) of each windows
 * @param left_points The returned points which were found in the window search on the left side 
 * @param right_points The returned points which were found in the window search on the right side 
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING
 * @note The output_points are stored the following way:
 */
int sliding_windows_search(Mat &input_img, const double roi, const int num_windows, const int width,
                           std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * Returns the (x or y) coordinates of the sub-domains
 * @param start of the whole domain to be partitioned
 * @param end of the whole domain to be partitioned
 * @param euqidistant whether or not sub-domain borders should be euqidistant
 * @param number of sub-domains
 * @param coords array of length number+1, returns the coordinates of the sub-domain borders (including start and end)
 */
void sub_partition(const int start, const int end, const int number, const bool equidistant, int *coords);

/**
 * ### Deprecated ###
 * Takes a binary image and fills it with black (=0) except for the vertical region of interest (roi) (-> along the y axis)
 * @param img is the image to be modified and returned
 * @param start is the vertical start where the roi is starting in percent [0;1]
 */
void v_roi(Mat &img, const int start);

/**
 * Given two input-window-starting-points (from a h_histogram), search those in three different places along the y-axis 
 * to find 6 total points of the two curved road lane
 * @param img image to be searched for the six road lane points
 * @param window_width width of one of the two search windows
 * @param roi Percent of the image rows where the region of interest start
 * @param output_points returns the six points found in the image. If no suitable point is found (-1,-1) is returned
 * @note can be prallelized for the six independent regions -> early aborting as soon as a point is found
 * @note points are saved from top (high y values) to bottom (low y values) in [left/right]_points
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING
 */
int window_search(const Mat &img, const int window_width, const double roi,
                  std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);
