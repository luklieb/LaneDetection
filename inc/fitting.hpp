#pragma once
#include <vector>
#include <string>
#include <opencv/cv.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "codes.hpp"
#include "helper.hpp"

using namespace cv;

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
 * @param b_inv Inverse perspective transform matrix (in order to go from Bird-View to normal-view)
 *        if empty (=default argument), then draw_poly() will perform no perspective transformation
 */
void draw_poly(Mat &image, const double roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff,
               const int order, const Mat &b_inv = Mat());

/**
 * Takes an input vector of points and does a polynomial regression on it
 * It always fits a polynomial of order = num_points-1
 * @param points Vector holding the points to be fit
 * @param coeff Return vector holding the num_points coefficients
 * @note See here: http://mathworld.wolfram.com/LeastSquaresFittingPolynomial.html
 */
void poly_reg(const std::vector<Point2f> &left_points, const std::vector<Point2f> &right_points,
              std::vector<double> &left_coeff, std::vector<double> &right_coeff, const int order);

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
 * @param b_inv Inverse perspective transform matrix (in order to go from Bird-View to normal-view)
 *        if empty (=default argument), then store_result() will perform no perspective transformation
 * @return Returns either LANEDET_SUCCESS or LANEDET_ERROR
 */
int store_result(const Mat &image, const double &roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff,
                 const int order, const String dir, const String file, const Mat &b_inv = Mat());

/**
 * If a LANEDET_WARNING is detected this function is called. It stores a plain red image to the specified location
 * This way the evaluation realizes and takes into account, that no lanes/roads were detected
 * @param image Used for correct output image sizes
 * @param dir Path where the red image should be stored
 * @param file File name of the resulting image
 */
void store_void_result(const Mat &image, const String dir, const String file);
