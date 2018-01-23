/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2014, Itseez, Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "algos.hpp"

using namespace cv;

/**
 * type used in function HoughLinesCustom
 */
struct LinePolar
{
    float rho;
    float angle;
};

/**
 * type used as functioanl in function HoughLinesCustom
 */
struct hough_cmp_gt
{
    hough_cmp_gt(const int *_aux) : aux(_aux) {}
    inline bool operator()(int l1, int l2) const
    {
        return aux[l1] > aux[l2] || (aux[l1] == aux[l2] && l1 < l2);
    }
    const int *aux;
};

/**
 * Returns rho and theta coordiantes of the found lines
 * Copied from github opencv/modules/imgproc/src/hough.cpp
 * @param img input image 
 * @param rho distance resolution parameter in pixels
 * @param theta angle resolution parameter in radiants
 * @param threshold minimum number of intersections to detect a line
 * @param lines vector returning the found lines (in rho and theta), at beginning of vector all linesMax left lines, then all linesMax right lines
 * @param linesMax number of lines to be found per side
 * @param h_roi start of the horizontal region of interest in percent [0;1]
 * @note be aware that if the current line is similar (parrallel) to the previous line it gets ignored
 *       -> might cause problems with "bird eye view" (because then both road lines are parallel)
 */
/**
 * @note: add further failsafe (one more bool to check wheter birdsview or not and then
 * add restrictions in function to restrict e.g. the angle of the second line, etc.)
 */
void HoughLinesCustom(const Mat &img, float rho, float theta,
                      int threshold, std::vector<Vec2f> &lines, int linesMax, int roi_start, int roi_end,
                      double min_theta, double max_theta)
{
    int i, j;
    float irho = 1 / rho;

    CV_Assert(img.type() == CV_8UC1);

    const uchar *image = img.ptr();
    int step = (int)img.step;
    int width = img.cols;
    //int height = img.rows;
    int height = roi_end - roi_start;

    if (max_theta < min_theta)
    {
        CV_Error(CV_StsBadArg, "max_theta must be greater than min_theta");
    }
    int numangle = cvRound((max_theta - min_theta) / theta);
    int numrho = cvRound(((width + height) * 2 + 1) / rho);

#if defined HAVE_IPP && IPP_VERSION_X100 >= 810 && !IPP_DISABLE_HOUGH
    CV_IPP_CHECK()
    {
        IppiSize srcSize = {width, height};
        IppPointPolar delta = {rho, theta};
        IppPointPolar dstRoi[2] = {{(Ipp32f) - (width + height), (Ipp32f)min_theta}, {(Ipp32f)(width + height), (Ipp32f)max_theta}};
        int bufferSize;
        int nz = countNonZero(img);
        int ipp_linesMax = std::min(linesMax, nz * numangle / threshold);
        int linesCount = 0;
        lines.resize(ipp_linesMax);
        IppStatus ok = ippiHoughLineGetSize_8u_C1R(srcSize, delta, ipp_linesMax, &bufferSize);
        Ipp8u *buffer = ippsMalloc_8u_L(bufferSize);
        if (ok >= 0)
        {
            ok = CV_INSTRUMENT_FUN_IPP(ippiHoughLine_Region_8u32f_C1R, image, step, srcSize, (IppPointPolar *)&lines[0], dstRoi, ipp_linesMax, &linesCount, delta, threshold, buffer);
        };
        ippsFree(buffer);
        if (ok >= 0)
        {
            lines.resize(linesCount);
            CV_IMPL_ADD(CV_IMPL_IPP);
            return;
        }
        lines.clear();
        setIppErrorStatus();
    }
#endif

    AutoBuffer<int> _accum((numangle + 2) * (numrho + 2));
    std::vector<int> _sort_buf_right;
    std::vector<int> _sort_buf_left;
    AutoBuffer<float> _tabSin(numangle);
    AutoBuffer<float> _tabCos(numangle);
    int *accum = _accum;
    float *tabSin = _tabSin, *tabCos = _tabCos;

    memset(accum, 0, sizeof(accum[0]) * (numangle + 2) * (numrho + 2));

    float ang = static_cast<float>(min_theta);
    for (int n = 0; n < numangle; ang += theta, n++)
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }

    // stage 1. fill accumulator
    for (i = roi_start; i < roi_end; i++)
        for (j = 0; j < width; j++)
        {
            if (image[i * step + j] != 0)
                for (int n = 0; n < numangle; n++)
                {
                    //radius = distance to line
                    int r = cvRound(j * tabCos[n] + i * tabSin[n]);
                    r += (numrho - 1) / 2;
                    accum[(n + 1) * (numrho + 2) + r + 1]++;
                }
        }

    // stage 2. find local maximums
    //horizontal lines (~0.5*pi = 90°) are excluded
    //0.2 and 0.8 for relatively steep (= vertical) lanes --> good for birdseye view
    for (int r = 0; r < numrho; r++)
    {
        for (int n = 0; n < 0.2 * numangle; n++)
        {
            int base = (n + 1) * (numrho + 2) + r + 1;
            if (accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                _sort_buf_left.push_back(base);
        }
        for (int n = 0.8 * numangle; n < numangle; n++)
        {
            int base = (n + 1) * (numrho + 2) + r + 1;
            if (accum[base] > threshold &&
                accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                _sort_buf_right.push_back(base);
        }
    }

    // stage 3. sort the detected lines by accumulator value
    std::sort(_sort_buf_left.begin(), _sort_buf_left.end(), hough_cmp_gt(accum));
    std::sort(_sort_buf_right.begin(), _sort_buf_right.end(), hough_cmp_gt(accum));

    // stage 4. store the first min(total,linesMax) lines to the output buffer
    //skips parallel lines within "range" of radiants angle
    //linesMax = std::min(linesMax, (int)_sort_buf.size());
    double scale = 1. / (numrho + 2);
    double last_angle = 0;
    double range = 0.3; //range in radiants to check for last_angle, 0.01 radiants = 0.57°
    i = 0;              //counter for numbers of actual line segments
    j = 0;              //traverses the _sort_buf array
    while (i < linesMax && j < (int)_sort_buf_left.size())
    {
        LinePolar line;
        int idx = _sort_buf_left[j];
        int n = cvFloor(idx * scale) - 1;
        int r = idx - (n + 1) * (numrho + 2) - 1;
        line.rho = (r - (numrho - 1) * 0.5f) * rho;
        line.angle = static_cast<float>(min_theta) + n * theta;
        std::cout << "i: " << i << ", last: " << last_angle << ", lineangle: " << line.angle << std::endl;
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
            std::cout << "continue" << std::endl;
            ++j;
            continue; //current line is too similar (parallel) to previous line -> skip it
        }
        last_angle = line.angle;
        lines.push_back(Vec2f(line.rho, line.angle));
        ++i;
        ++j;
    }
    i = 0;
    j = 0;
    while (i < linesMax && j < (int)_sort_buf_right.size())
    {
        LinePolar line;
        int idx = _sort_buf_right[j];
        int n = cvFloor(idx * scale) - 1;
        int r = idx - (n + 1) * (numrho + 2) - 1;
        line.rho = (r - (numrho - 1) * 0.5f) * rho;
        line.angle = static_cast<float>(min_theta) + n * theta;
        std::cout << "i: " << i << ", last: " << last_angle << ", lineangle: " << line.angle << std::endl;
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
            std::cout << "continue" << std::endl;
            ++j;
            continue; //current line is too similar (parallel) to previous line -> skip it
        }
        last_angle = line.angle;
        lines.push_back(Vec2f(line.rho, line.angle));
        ++i;
        ++j;
    }
}

/**
 * Takes a binary image and fills it with black (=0) except for the vertical region of interest (roi) (-> along the y axis)
 * @param img is the image to be modified and returned
 * @param start is the vertical start where the roi is starting in percent [0;1]
 */
void v_roi(Mat &img, const double &start)
{
    rectangle(img, Point(0, 0), Point(img.cols, img.rows * start), Scalar(0), CV_FILLED);
}

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
void bird_view(const Mat &input_img, Mat &output_img, double rel_height, double rel_left, double rel_right, Mat *transform)
{
    double offset = 0.4;
    double offset2 = 0.05;
    Point2f p1[4] = {Point2f(rel_left * input_img.cols, rel_height * input_img.rows), Point2f(rel_right * input_img.cols, rel_height * input_img.rows), Point2f((rel_right + offset) * input_img.cols, input_img.rows), Point2f((rel_left - offset) * input_img.cols, input_img.rows)};
    //Point2f p2[4] = {Point2f(0, 0), Point2f(input_img.cols, 0), Point2f(input_img.cols, input_img.rows), Point2f(0, input_img.rows)};
    Point2f p2[4] = {Point2f((rel_left - offset2) * input_img.cols, 0), Point2f((rel_right + offset2) * input_img.cols, 0), Point2f((rel_right + offset2) * input_img.cols, input_img.rows), Point2f((rel_left - offset2) * input_img.cols, input_img.rows)};

    Mat mat = getPerspectiveTransform(p1, p2);
    warpPerspective(input_img, output_img, mat, Size(input_img.cols, input_img.rows));
}

/**
 * Returns the two horizontal (x-coordinates) points (one for right/left half) with the maximum according to the histogram
 * @param input_img is the binary input image
 * @param points holds the 2 x-coordinates according to the two max values of the histogram
 * @note reduce() function could be optimized with an additional roi constraint
 */
void h_histogram(const Mat &input_img, int *x_points)
{
    std::vector<int> histo;
    reduce(input_img, histo, 0, CV_REDUCE_SUM);
    int m1 = 0;
    int m2 = 0;
    x_points[0] = -1;
    x_points[0] = -1;
    for (size_t i = 0; i < 0.5 * histo.size(); ++i)
    {
        if (histo[i] > m1)
        {
            m1 = histo[i];
            x_points[0] = i;
        }
    }
    for (size_t i = 0.5 * histo.size(); i < histo.size(); ++i)
    {
        if (histo[i] > m2)
        {
            m2 = histo[i];
            x_points[1] = i;
        }
    }
}

/**
 * Takes the histogram of the upper half of the image as a startig point for 
 * a windows search along the road lanes.
 * @param input_img Image to be searched for road lanes
 * @param num_windows The number of windows for both lanes (left and right)
 * @param width The width (in x-direction) of each windows
 * @param points The returned points which were found in the window search. 
 * @note The output_points are stored the following way:
 *      first, all num_windows left lane points are stored (from top to bottom),
 *      followed by all num_windows right lane points:
 * 
 *      num_windows-1   2*num_windows-1
 *      ...             ...
 *      1               num_window+1
 *      0               num_windows
 */

void multiple_windows_search(Mat &input_img, const int num_windows, const int width, std::vector<Point2f> &output_points)
{

    int upper_histo[2];
    std::vector<Point> non_zero_left;
    std::vector<Point> non_zero_right;
    //only take upper half of input_img into consideration
    std::cout << "histo " << std::endl;
    h_histogram(Mat(input_img, Rect(0, 0.5 * input_img.rows, input_img.cols, 0.5 * input_img.rows)), upper_histo);
    std::cout << "histo done: " << upper_histo[0] << ", " << upper_histo[1] << std::endl;
    //offsets from center points of window
    const int height = input_img.rows / num_windows;
    const int y_offset = 0.5 * height;
    const int x_offset = 0.5 * width;
    int x_tmp = 0;
    int x = upper_histo[0];
    assert(x - x_offset > 0 && x - x_offset + width < input_img.cols);
    int y_tmp = 0;
    int y = input_img.rows - y_offset;

    //left points
    for (int i = 0; i < num_windows; ++i)
    {
        findNonZero(Mat(input_img, Rect(x - x_offset, y - y_offset, width, height)), non_zero_left);

#ifndef NDEBUG
//rectangle(input_img, Rect(x-x_offset, y-y_offset, width-2, height-2), Scalar(255));
#endif

        x_tmp = 0;
        y_tmp = 0;

        if (non_zero_left.size() > 0)
        {
            //averages all coordinates of non-zero pixels --> new output coordinates
            //x_tmp becomes center of new search window
            for (auto &p : non_zero_left)
            {
                x_tmp += p.x + x - x_offset;
                y_tmp += p.y + y - y_offset;
            }
            x_tmp /= non_zero_left.size();
            y_tmp /= non_zero_left.size();
        }
        //if no white pixels (lanes) are found, then just use the former coordinates to create current coordinates
        else
        {
            //if enough points are already found, use the previous 2 to linearly extrapolate to the current point
            if (i >= 2)
            {
                double slope = (output_points[i - 1].x - output_points[i - 2].x) / (output_points[i - 1].y - output_points[i - 2].y);
                y_tmp = y;
                x_tmp = x - (height * slope);
            }
            //if not, move up in a straight line
            else
            {
                x_tmp = x;
                y_tmp = y;
            }
        }

#ifndef NDEBUG
//line(input_img, Point(x_tmp, y_tmp), Point(x_tmp, y_tmp), Scalar(255));
#endif

        output_points.push_back(Point2f(x_tmp, y_tmp));

        //move search window one step down
        y -= height;
        x = x_tmp;

        non_zero_left.clear();
    }

    //right points analogous
    y = input_img.rows - y_offset;
    x = upper_histo[1];
    x_tmp = 0;
    assert(x - x_offset > 0 && x - x_offset + width < input_img.cols);
    for (int i = 0; i < num_windows; ++i)
    {
        findNonZero(Mat(input_img, Rect(x - x_offset, y - y_offset, width, height)), non_zero_right);

#ifndef NDEBUG
//rectangle(input_img, Rect(x-x_offset, y-y_offset, width-1, height-2), Scalar(255));
#endif

        x_tmp = 0;
        y_tmp = 0;

        if (non_zero_right.size() > 0)
        {
            for (auto &p : non_zero_right)
            {
                x_tmp += p.x + x - x_offset;
                y_tmp += p.y + y - y_offset;
            }
            x_tmp /= non_zero_right.size();
            y_tmp /= non_zero_right.size();
        }
        else
        {
            if (i >= 2)
            {
                double slope = (output_points[num_windows + i - 1].x - output_points[num_windows + i - 2].x) / (output_points[num_windows + i - 1].y - output_points[num_windows + i - 2].y);
                y_tmp = y;
                x_tmp = x - (height * slope);
            }
            else
            {
                x_tmp = x;
                y_tmp = y;
            }
        }

#ifndef NDEBUG
//line(input_img, Point(x_tmp, y_tmp), Point(x_tmp, y_tmp), Scalar(255), 2, 8, 0);
#endif

        output_points.push_back(Point2f(x_tmp, y_tmp));

        y -= height;
        x = x_tmp;

        non_zero_right.clear();
    }
}

/**
 * Given two input-window-starting-points (from a h_histogram), search those in three different places along the y-axis 
 * to find 6 total points of the two curved road lane
 * @param img image to be searched for the six road lane points
 * @param input_points holds the two window starting points from a h_histogram
 * @param window_width width of one of the two search windows
 * @param output_points returns the six points found in the image. If no suitable point is found (-1,-1) is returned
 * @note can be prallelized for the six independent regions -> early aborting as soon as a point is found
 * @note points are saved in output_points as follows (first all left lane points (from top to bottom),
 *      then all right left lane points (from top to bottom):   
 *      2 5
 *      1 4
 *      0 3
 */
void window_search(const Mat &img, const int *input_points, const int &window_width, std::vector<Point> &output_points)
{
    for (int i = 0; i < 6; ++i)
    {
        output_points[i] = Point(-1, -1);
    }
    const uchar *image = img.ptr();
    int low = 0.1 * img.rows, mid = 0.5 * img.rows, up = 0.9 * img.rows;
    const int offset = 50;
    assert((low - offset >= 0) && (up + offset < img.rows));

    for (int r = -offset; r < offset; ++r)
    {
        for (int c = -window_width; c < window_width; ++c)
        {
            if (image[(low + r) * img.cols + input_points[0] + c] == 255 && output_points[2].x == -1)
                output_points[2] = Point(input_points[0] + c, (low + r));
            if (image[(low + r) * img.cols + input_points[1] + c] == 255 && output_points[5].x == -1)
                output_points[5] = Point(input_points[1] + c, (low + r));
            if (image[(mid + r) * img.cols + input_points[0] + c] == 255 && output_points[1].x == -1)
                output_points[1] = Point(input_points[0] + c, (mid + r));
            if (image[(mid + r) * img.cols + input_points[1] + c] == 255 && output_points[4].x == -1)
                output_points[4] = Point(input_points[1] + c, (mid + r));
            if (image[(up + r) * img.cols + input_points[0] + c] == 255 && output_points[0].x == -1)
                output_points[0] = Point(input_points[0] + c, (up + r));
            if (image[(up + r) * img.cols + input_points[1] + c] == 255 && output_points[3].x == -1)
                output_points[3] = Point(input_points[1] + c, (up + r));
        }
    }
}

/**
 * Takes three points, fits a quadratic curve to them and draws the curve on the image
 * @param image to be drawn on
 * @param points holding three points
 * @param start is index of first Point in points to be considered
 */
void draw_curve(Mat &image, const std::vector<Point> &points, const uint start)
{
    assert(points.size() >= 3 + start);
    uchar *img = image.ptr();
    Mat lhs = (Mat_<double>(3, 3) << points[0 + start].y * points[0 + start].y, points[0 + start].y, 1.,
               points[1 + start].y * points[1 + start].y, points[1 + start].y, 1.,
               points[2 + start].y * points[2 + start].y, points[2 + start].y, 1.);

    Mat rhs = (Mat_<double>(3, 1) << points[0 + start].x, points[1 + start].x, points[2 + start].x);

    Mat solution = Mat_<double>();

    solve(lhs, rhs, solution);

    const double *sol = solution.ptr<double>();
    for(int i = 0; i < points.size()-start; ++i)
        std::cout << sol[i] << ", " << std::endl;
    for (int r = 0; r < image.rows; ++r)
        img[r * image.cols + static_cast<int>((sol[0] * r * r + sol[1] * r + sol[2]))] = 128;
}

/**
 * Takes an input vector of points and does a polynomal regression on it
 * It always fits a polynomial of order = num_points-1
 * @param points Vector holding the points to be fit
 * @param coeff Return vector holding the num_points coefficients
 * @note See here: http://mathworld.wolfram.com/LeastSquaresFittingPolynomial.html
 */
void poly_reg(const std::vector<Point> &points, std::vector<double> &coeff)
{
    const int num_points = points.size();
    Mat lhs = Mat_<double>(num_points, num_points);
    Mat rhs = Mat(num_points, 1, CV_64F);
    Mat solution = Mat_<double>();

    //constructs simple matrix (not Vandermonde matrix)
    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j < num_points; ++j)
        {
            lhs.at<double>(i, j) = std::pow((double)points[i].x, j);
        }
    }

    for (int i = 0; i < num_points; ++i)
        rhs.at<double>(i) = points[i].y;

    solve(lhs, rhs, solution);

    const double *sol = solution.ptr<double>();
    for(int i = 0; i < num_points; ++i)
        coeff.push_back(sol[i]);
}