#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "codes.hpp"
#include "helper.hpp"

using namespace cv;

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
 * @param thres Lower threshold
 * @param kernel Size of kernel
 * @note compare to gabor(), sobel_thres_*() and color_thres()
 */
void canny_blur(Mat &image, const int thres, const int kernel);

/**
 * Edge detection by color thresholding in the HLS color space
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres Start threshold value for the L channel
 */
void color_thres(Mat &image, const int thres = 220);

/**
 * @note DEPRECATED, couldn't find any configurations for its arguments to make it work decently
 * Creates a Gabor kernel and applies it to the input image for edge detection
 * @param image being converted to an edge image
 * @note compare to canny_blur(Mat&)
 */
void gabor(Mat &image);

/**
 * Removes only  horizontal lines in the input image 
 * @param image in which horizontal lines should be removed
 */
void h_sobel(Mat &image);

/**
 * Applies multiple independend edge detection filters to image
 * @param image Input color image. Returns a binary (CV_8U) Mat with the detected edges in white
 * @param algos Vector with numbers representing edge detection algorithms; The order is not important
 * @param ca_thres Threshold for canny edge around (150-240)
 * @param kernel Kernel size (3,5,7)
 * @param s_mag Threshold for Sobel magnitude around (150-240)
 * @param s_par_x Threshold for Sobel derivative in x direction around (10-100)
 * @param s_par_y Threshold for Sobel derivative in y direction around (50-150)
 * @param c_thres Threshold for L-channel of HLS color space around (180-240)
 * @note 1 = canny, 2 = sobel_mag_thres, 3 = sobel_par_thres, 4 = color_thres
 * @note each number should only exit max. once in vector algos
 */
void multi_filter(Mat &image, std::vector<int> algos, int ca_thres, int kernel, int s_mag, int s_par_x, int s_par_y, int c_thres);

/**
 * @note DEPRECATED, because results were not good enough. No direct replacement
 * Edge detection by Sobel derivativ direction thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres_1 First threshold in degrees. Directions +- 5° around it are searched
 * @param thres_2 Second threshold in degrees. Directions +- 5° around it are searched
 */
void sobel_dir_thres(Mat &image, const int thres_1 = 90, const int thres_2 = 180);

/**
 * Edge detection by Sobel magnitude thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres Threshold for the magnitude
 */
void sobel_mag_thres(Mat &image, const int thres = 50);

/**
 * Edge detection by Sobel partial derivative thresholding
 * @param image Input image for edge detection. Returns detected edges as a binary (one channel) image
 * @param thres_x Start threshold value for the derivatives in x direction
 * @param thres_y Start threshold value for the derivatives in y direction
 */
void sobel_par_thres(Mat &image, const int thres_x = 10, const int thres_y = 60);