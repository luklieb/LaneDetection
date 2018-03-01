#pragma once
#include <opencv/cv.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/core/cv_cpu_helper.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include <string>

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
 * MAPRA_WARNING => Detected lanes probably wrong
 * MAPRA_SUCCESS => Actual lanes were found
 * MAPRA_ERROR => Severe Error 
 */
#define MAPRA_WARNING -1
#define MAPRA_SUCCESS 0
#define MAPRA_ERROR 1


/**
 * Conducts Acitve Line Modelling (ALM) on the input points (belonging pariwise to lines from a Hough transform after get_points())
 * @note Needs for each side num_part * num_lines * 2 points; Start points of one line segment have even indices, end points have odd indices
 * @param left_points Holds the pairwise points (start and end of one line) of the lines belonging to the left lane of the different partitionts
 * @param right_points Holds the pairwise points (start and end of line) of the lines belonging to the right lane of the different partitions 
 * @param num_part Number of partitions per side
 * @param num_lines Number of lines per partition
 * @return Returns either MAPRA_WARNING or MAPRA_SUCESS
 */
int alm(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part, const int num_lines);

/**
 * Converts the points-pairs making up lines returned from alm() into single points (in order to compute the coefficients)
 * It computes the x-cooordinate average of two points with the same y-coordinate (two adjacent lines)
 * @param left_points Holds the points-pairs for the left lane
 * @param left_points Holds the points-pairs for the right lane
 * @return Returns either MAPRA_WARNING or MAPRA_SUCESS
 */
int alm_conversion(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

/**
 * @note  ### Deprecated ###
 * Transforms the trapezoid (p1) of the input image to a rectangle (p2) in the output image creating a top-down 'bird view' image
 * @param input_img input image
 * @param output_img empty image with same size as input_img
 * @param rel_height is the relative height of the to be transformed image section (number between 0 and 1) 
 * @param rel_left is the relative horizontal distance from the origin to the upper left corner of the trapezoid (number between 0 and 1)
 * @param rel_right is the relative horizontal distance from the origin to the upper right corner of the trapezoid (number between 0 and 1)
 * @note p1 has to be modified for different cameras and positions of the camera
 * @note after transformation edges of lines get blurry --> might be a problem for canny edge detection
 */
void bird_view(const Mat &input_img, Mat &output_img, const double rel_height, const double rel_left, const double rel_right);

/**
 * Blurs the input image and applies Canny edge detection on it 
 * @param image being converted to an edge image
 * @note compare to gabor(Mat&)
 */
void canny_blur(Mat &image);


/**
 * Edge detection by color thresholding in the HLS color space
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres Start threshold value for the L channel
 */
void color_thres(Mat &image, const int thres = 220);

/**
 * @note Deprecated, replaced by poly_reg()
 * Takes three points, fits a quadratic curve to them and draws the curve on the image
 * @param image to be drawn on
 * @param roi Vertical starting point in percent of the region of interest
 * @param points holding three points
 */
void draw_curve(Mat &image, const double roi, const std::vector<Point> &left_points, const std::vector<Point> &right_points);

/**
 * Draws the two polynomials (of order [left/right]_coeff.size()-1) according to their coefficients on the input image
 * @param image Image to be drawn on
 * @param roi Vertical starting point in percent of the region of interest
 * @param left_coeff Holds the coefficients for the left lane polynomial
 * @param right_coeff Holds the coefficients for the right lane polynomial
 * @param order Order of polynomial (e.g. for quadradic polynomials the order is 2)
 */
void draw_poly(Mat &image, const double roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order);

/**
 * Creates a Gabor kernel and applies it to the input image for edge detection
 * @param image being converted to an edge image
 * @note compare to canny_blur(Mat&)
 */
void gabor(Mat &image);

/**
 * Converts the polar coordiantes from a Hough Transform (lines) to points according to their start/end of each partition
 * @param lines The polar coordinates returned from a Hough Transform
 * @param num_lines The number of lines of each side of each partition
 * @param num_part The number of partitions
 * @param coords_part Coordinates of the partitions
 * @param points Holds the converted Points 
 * @return Returns either MAPRA_SUCCESS or MAPRA_WARNING
 * @note Storage order for both [left/right]_points: points belonging to same line are stored consecutively:
 * 		Point_0(line_0), Point_1(line_0), Point_2(line_1), Point_3(line_1), ..., Point_num_lines*2-1(line_num_lines-1), ..., Point_num_lines*num_part*2-1(line_num_lines*num_part-1) 
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
 * Removes only  horizontal lines in the input image 
 * @param image in which horizontal lines should be removed
 */
void h_sobel(Mat &image);

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
int HoughLinesCustom(const Mat &img, const float rho, const float theta,
                      const int threshold, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines,
                      const int linesMax, const int roi_start, const int roi_end, const bool b_view, const double min_theta = 0, const double max_theta = CV_PI);

/**
 * Takes the histogram of the upper half of the image as a startig point for 
 * a windows search along the road lanes.
 * @param input_img Image to be searched for road lanes
 * @param roi Vertical starting point in percent of the region of interest (for the h_histrogram() call within)
 * @param num_windows The number of windows for both lanes (left and right)
 * @param width The width (in x-direction) of each windows
 * @param left_points The returned points which were found in the window search on the left side 
 * @param right_points The returned points which were found in the window search on the right side 
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING
 * @note The output_points are stored the following way:
 */
int multiple_windows_search(Mat &input_img, const double roi, const int num_windows, const int width,
                             std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);

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
 * Takes an input vector of points and does a polynomal regression on it
 * It always fits a polynomial of order = num_points-1
 * @param points Vector holding the points to be fit
 * @param coeff Return vector holding the num_points coefficients
 * @note See here: http://mathworld.wolfram.com/LeastSquaresFittingPolynomial.html
 */
void poly_reg(const std::vector<Point2f> &left_points, const std::vector<Point2f> &right_points, 
                std::vector<double> &left_coeff, std::vector<double> &right_coeff, const int order);


/**
 * Helper function to show an image
 * @param image_name for the window
 * @param image to be shwon in window
 * @param wait option to wait for a key input to close the window showing the image
 */
void show_image(const String image_name, const Mat &image, const bool wait);

/**
 * Edge detection by Sobel derivativ direction thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres_s First threshold in degrees. Directions +- 5° around it are searched
 * @param thres_e Second threshold in degrees. Directions +- 5° around it are searched
 */
void sobel_dir_thres(Mat &image, const int thres_s = 90, const int thres_e = 180);

/**
 * Edge detection by Sobel magnitude thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres Threshold for the magnitude
 */
void sobel_mag_thres(Mat &image, const int thres = 155);

/**
 * Edge detection by Sobel derivative thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres_x Start threshold value for the derivatives in x direction
 * @param thres_y Start threshold value for the derivatives in y direction
 */
void sobel_thres(Mat &image, const int thres_x = 10, const int thres_y = 60);


/**
 * Saves an color coded image after lane detection for the evaluation in file evaluate.py
 * @note OpenCV uses BGR -> non-lane areas colored in red => (0,0,255), lane areas in pink => (255,0,255) 
 * @param image Used for correct output image sizes
 * @param roi Vertical starting point in percent of the region of interest
 * @param left_coeff Coefficients for the polynomial describing the left boundary of the lane
 * @param right_coeff Coefficients for the polynomial describing the right boundary of the lane
 * @param order Order of polynomials
 * @param dir Path where the resulting image should be stored
 * @param file File name of the resulting image
 * @param Returns either MAPRA_SUCCESS or MAPRA_ERROR
 */
int store_result(const Mat &image, const double &roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order, const String dir, const String file);


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
 * @param input_points holds the two window starting points from a h_histogram
 * @param window_width width of one of the two search windows
 * @param roi Percent of the image rows where the region of interest start
 * @param output_points returns the six points found in the image. If no suitable point is found (-1,-1) is returned
 * @note can be prallelized for the six independent regions -> early aborting as soon as a point is found
 * @note points are saved from top (high y values) to bottom (low y values) in [left/right]_points
 * @return Returns either MAPRA_SUCCESS, MAPRA_WARNING
 */
int window_search(const Mat &img, const int *input_points, const int window_width, const double roi,
                   std::vector<Point2f> &left_points, std::vector<Point2f> &right_points);
