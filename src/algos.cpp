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

//#############################################################################################
//######################################### HELPERS ###########################################
//#############################################################################################

/**
 * @note Y-value of points could be ignored, because they both should be on the same horizontal axis
 */
static inline double norm(const Point2f &p1, const Point2f &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

/**
 * type used in function HoughLinesCustom()
 */
struct LinePolar
{
    float rho;
    float angle;
};

/**
 * type used as functional in function HoughLinesCustom()
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

//#############################################################################################
//######################################### STATIC FCTS #######################################
//#############################################################################################

static void alm_(std::vector<Point2f> &points, const int &num_part, const int &num_lines)
{
    assert(points.size() % 2 == 0);
    //assert(points.size() == 2u * num_part * num_lines);

    std::vector<std::vector<Point2f>> possible_points(num_lines);
    double error_sum[num_lines];
    double error_tmp;
    int line_tmp;
    Point2f start_pt;
    Point2f end_pt;

    for (int l = 0; l < num_lines; ++l)
    {
        error_sum[0] = 0.;
        start_pt = points[l * 2];
        end_pt = points[l * 2 + 1];
        possible_points[l].push_back(start_pt);
        possible_points[l].push_back(end_pt);
        for (int p = 1; p < num_part; ++p)
        {
            error_tmp = std::numeric_limits<double>::infinity();
            line_tmp = -1;
            for (int l_new = p * num_lines; l_new < (p + 1) * num_lines; ++l_new)
            {
                start_pt = points[l_new * 2];
                if (norm(possible_points[l].back(), start_pt) < error_tmp)
                {
                    error_tmp = norm(possible_points[l].back(), start_pt);
                    error_sum[l] += error_tmp;
                    line_tmp = l_new;
                }
            }
            assert(line_tmp != -1);
            possible_points[l].push_back(points[line_tmp * 2]);
            possible_points[l].push_back(points[line_tmp * 2 + 1]);
        }
    }
    error_tmp = std::numeric_limits<double>::infinity();
    int index_tmp = -1;
    for (int i = 0; i < num_lines; ++i)
    {
        if (error_sum[i] < error_tmp)
        {
            error_tmp = error_sum[i];
            index_tmp = i;
        }
    }
    points.clear();
    points.insert(points.begin(), possible_points[index_tmp].begin(), possible_points[index_tmp].end());
}

void alm_conversion_(std::vector<Point2f> &points)
{
    //copy
    std::vector<Point2f> cpy(points);
    int size = points.size();
    assert(size % 2 == 0);
    points.clear();
    //first point can stay
    points.push_back(cpy[0]);
    //compute mean of x-coordinates of adjacent lines
    for (int i = 1; i < size - 1; i += 2)
    {
        points.push_back(Point2f(0.5 * (cpy[i].x + cpy[i + 1].x), cpy[i].y));
    }
    //last point can stay
    points.push_back(cpy[size - 1]);
}

//deprecated
static void draw_curve_(Mat &image, const std::vector<Point> &points)
{
    assert(points.size() >= 3);
    uchar *img = image.ptr();
    Mat lhs = (Mat_<double>(3, 3) << points[0].y * points[0].y, points[0].y, 1.,
               points[1].y * points[1].y, points[1].y, 1.,
               points[2].y * points[2].y, points[2].y, 1.);

    Mat rhs = (Mat_<double>(3, 1) << points[0].x, points[1].x, points[2].x);

    Mat solution = Mat_<double>();

    solve(lhs, rhs, solution);

    const double *sol = solution.ptr<double>();
    for (unsigned int i = 0; i < points.size(); ++i)
        std::cout << sol[i] << ", " << std::endl;
    for (int r = 0; r < image.rows; ++r)
        img[r * image.cols + static_cast<int>((sol[0] * r * r + sol[1] * r + sol[2]))] = 128;
}

static void draw_poly_(Mat &image, const std::vector<double> &coeff, const int order)
{
    assert(coeff.size() == (unsigned int)order+1);
    double column = 0.;
    for (int r = 0; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            column += std::pow(r, c) * coeff[c];
        }
        image.at<uchar>(r, static_cast<int>(column)) = 255;
        column = 0.;
    }
}

static void get_points_(const std::vector<Vec2f> &lines, const int &num_lines, const int *coords_part, std::vector<Point2f> &points)
{
    points.clear();
    int i = 0;
    int j = 0;
    Point2f pt1, pt2;
    for (auto p : lines)
    {
        //std::cout << p[0] << ", " << p[1] / CV_PI << std::endl;
        float rho = p[0], theta = p[1];
        double a = cos(theta), b = sin(theta), a_inv = 1. / a;
        pt1.y = coords_part[i];
        pt1.x = (rho - pt1.y * b) * a_inv;
        pt2.y = coords_part[i + 1];
        pt2.x = (rho - pt2.y * b) * a_inv;
        points.push_back(pt1);
        points.push_back(pt2);
        ++j;
        if (j % num_lines == 0)
            ++i;
    }
}

static void multiple_windows_search_(Mat &input_img, const int num_windows, const int width, std::vector<Point2f> &points, const bool left)
{

    int upper_histo[2];
    points.clear();
    //has to be of type Point
    std::vector<Point> non_zero;
    //only take upper half of input_img into consideration
    std::cout << "histo " << std::endl;
    h_histogram(Mat(input_img, Rect(0, 0.5 * input_img.rows, input_img.cols, 0.5 * input_img.rows)), upper_histo);
    std::cout << "histo done: " << upper_histo[0] << ", " << upper_histo[1] << std::endl;
    //offsets from center points of window
    const int height = input_img.rows / num_windows;
    const int y_offset = 0.5 * height;
    const int x_offset = 0.5 * width;
    int x_tmp = 0;
    int x;
    if (left)
    {
        x = upper_histo[0];
        if(x-x_offset < 0)
            x = x_offset;
    }
    else
    {
        x = upper_histo[1];
        if(x+x_offset >= input_img.cols)
            x = input_img.cols - 1 - x_offset;
    }

    assert(x - x_offset >= 0);
    assert(x - x_offset + width < input_img.cols);
    int y_tmp = 0;
    int y = input_img.rows - y_offset;

    for (int i = 0; i < num_windows; ++i)
    {
        //find all indizes of white pixels --> so we can compute the mean of them later
        std::cout << "nonzero" << " x: " << x << ", y: " << y << std::endl;
        findNonZero(Mat(input_img, Rect(x - x_offset, y - y_offset, width, height)), non_zero);
        std::cout << "nonzero done" << std::endl;
#ifndef NDEBUG
//rectangle(input_img, Rect(x-x_offset, y-y_offset, width-2, height-2), Scalar(255));
#endif

        x_tmp = 0;
        y_tmp = 0;

        if (non_zero.size() > 0)
        {
            //averages all coordinates of non-zero pixels --> new output coordinates
            //x_tmp becomes center of new search window
            for (auto &p : non_zero)
            {
                x_tmp += p.x + x - x_offset;
                y_tmp += p.y + y - y_offset;
            }
            x_tmp /= non_zero.size();
            y_tmp /= non_zero.size();
        }
        //if no white pixels (lanes) are found, then just use the former coordinates to create current coordinates
        else
        {
            //if enough points are already found, use the previous 2 to linearly extrapolate to the current point
            if (i >= 2)
            {
                double slope = (points[i - 1].x - points[i - 2].x) / (points[i - 1].y - points[i - 2].y);
                y_tmp = y;
                x_tmp = x - (height * slope);
            }
            //if not, move up in a straight line
            else
            {
                x_tmp = x;
                y_tmp = y;
            }
            std::cout << "multiple_window no white pixels found" << std::endl;
        }

#ifndef NDEBUG
//line(input_img, Point(x_tmp, y_tmp), Point(x_tmp, y_tmp), Scalar(255));
#endif

        points.push_back(Point2f(x_tmp, y_tmp));

        //move search window one step down
        y -= height;
        x = x_tmp;
        //if new search coordinates are not in the image, set them back to the image
        if(x+x_offset >= input_img.cols)
            x = input_img.cols - 1 - x_offset;
        if(x-x_offset < 0)
            x = x_offset;

        non_zero.clear();
    }
}

static void poly_reg_(const std::vector<Point2f> &points, std::vector<double> &coeff, const int order)
{
    assert(points.size() >= (unsigned int)order + 1);
    coeff.clear();
    const int num_points = points.size();
    Mat lhs = Mat_<double>(num_points, order+1);
    Mat rhs = Mat(num_points, 1, CV_64F);
    Mat solution = Mat_<double>();

    //constructs simple matrix (not Vandermonde matrix)
    for (int i = 0; i < num_points; ++i)
    {
        for (int j = 0; j <= order; ++j)
        {
            lhs.at<double>(i, j) = std::pow((double)points[i].y, j);
        }
    }

    for (int i = 0; i < num_points; ++i)
        rhs.at<double>(i) = points[i].x;

    solve(lhs, rhs, solution, DECOMP_QR);

    const double *sol = solution.ptr<double>();
    //coeff.resize(num_points);
    //coeff.insert(coeff.begin(), sol[0], sol[num_points-1]);
    for (int i = 0; i <= order; ++i)
        coeff.push_back(sol[i]);
}

//#############################################################################################
//######################################### INTERFACE #########################################
//#############################################################################################

void alm(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int &num_part, const int &num_lines)
{
    alm_(left_points, num_part, num_lines);
    alm_(right_points, num_part, num_lines);
}

void alm_conversion(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    alm_conversion_(left_points);
    alm_conversion_(right_points);
}

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

void canny_blur(Mat &image)
{
    //original: low_threshold=50, kernel_size=3: smooth, but many left out edges
    //birdview: 350, 5
    int low_threshold = 100;
    int kernel_size = 3;
    blur(image, image, Size(3, 3));
    Canny(image, image, low_threshold, low_threshold * 3, kernel_size);
}

//deprecated
void draw_curve(Mat &image, const std::vector<Point> &left_points, const std::vector<Point> &right_points)
{
    draw_curve_(image, left_points);
    draw_curve_(image, right_points);
}

void draw_poly(Mat &image, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order)
{
    draw_poly_(image, left_coeff, order);
    draw_poly_(image, right_coeff, order);
}

void gabor(Mat &image)
{
    //TODO passende parameter finden
    int kernel_size = 50;
    double sigma = 5;         //frequency bandwidth
    double theta = 0.;        //angle of gabor kernel --> 0 for vertical lines
    double lambda = 20.;      //double the lane width in pixles
    double gamma = 0.;        //shape (circular or elliptical)
    double psi = 0.5 * CV_PI; //phase offset
    Mat kernel = getGaborKernel(Size(kernel_size, kernel_size), sigma, theta, lambda, gamma, psi, CV_32F);
    //resize(kernel, kernel, Size(500,500), 0, 0);
    //Mat img;
    //normalize(kernel, img, 0, 1, NORM_MINMAX);
    //show_image("bla", img, true);
    filter2D(image, image, CV_64F, kernel);
}

void get_points(const std::vector<Vec2f> &left_lines, const std::vector<Vec2f> &right_lines, const int &num_lines, const int *coords_part, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    get_points_(left_lines, num_lines, coords_part, left_points);
    get_points_(right_lines, num_lines, coords_part, right_points);
}

void h_histogram(const Mat &input_img, int *x_points)
{
    std::vector<int> histo;
    //"sums up" along y-axis for each column -> returns a "row vector"
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

void h_sobel(Mat &image)
{
    //consider only horizontal edges
    Sobel(image, image, -1, 1, 0);
}

void HoughLinesCustom(const Mat &img, float rho, float theta, int threshold,
                      std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines,
                      int lines_max, int roi_start, int roi_end, const bool b_view, double min_theta, double max_theta)
{

    //Additional parameters, that can be changed, but make everything way more complicated
    //Tuning them is too much work for this scope
    //filters out relativley horizontal lines (only accepts "steep" angles between [0, angle_roi*pi] and [(1-angle_roi)*pi, pi])
    const double angle_roi = 0.4;
    //Similar to angle_roi, but now only for b_view == true
    //Since the searched for lanes in the Birdview perspective are steeper, b_angle_roi < angle_roi
    const double b_angle_roi = 0.1;
    //only needed if b_view == true. Relative width of one side to look for lines for the respective left or right lane. 
    //Left lane is searched in [0, b_roi_widht*width]; Right lanes is searched in [(1-b_roi_width)*width, widht].
    const double b_roi_width = 0.55;

    int i, j;
    float irho = 1 / rho;

    CV_Assert(img.type() == CV_8UC1);

    const uchar *image = img.ptr();
    int step = (int)img.step;
    int width = img.cols;
    int height = roi_end - roi_start;

    if (max_theta < min_theta)
    {
        CV_Error(CV_StsBadArg, "max_theta must be greater than min_theta");
    }
    int numangle = cvRound((max_theta - min_theta) / theta);
    int numrho = cvRound(((width + height) * 2 + 1) / rho);

//opencv stuff --> can be ignored
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

    //_accum_addtional used for b_view == true optimizations
    //b_view == false uses only _accum (yes, then we waste the space for _accum_additional...)
    AutoBuffer<int> _accum((numangle + 2) * (numrho + 2));
    AutoBuffer<int> _accum_additional((numangle + 2) * (numrho + 2));
    std::vector<int> _sort_buf_right;
    std::vector<int> _sort_buf_left;
    AutoBuffer<float> _tabSin(numangle);
    AutoBuffer<float> _tabCos(numangle);
    int *accum = _accum;
    int *accum_additional = _accum_additional;
    float *tabSin = _tabSin, *tabCos = _tabCos;

    memset(accum, 0, sizeof(accum[0]) * (numangle + 2) * (numrho + 2));
    memset(accum_additional, 0, sizeof(accum_additional[0]) * (numangle + 2) * (numrho + 2));

    float ang = static_cast<float>(min_theta);
    for (int n = 0; n < numangle; ang += theta, n++)
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }

    // stage 1. fill accumulator
    if (b_view)
    {
        //left side in accum
        for (i = roi_start; i < roi_end; i++)
            for (j = 0; j < b_roi_width*width; j++)
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
        //right side in accum_additional
        for (i = roi_start; i < roi_end; i++)
            for (j = (1.-b_roi_width)*width; j < width; j++)
            {
                if (image[i * step + j] != 0)
                    for (int n = 0; n < numangle; n++)
                    {
                        //radius = distance to line
                        int r = cvRound(j * tabCos[n] + i * tabSin[n]);
                        r += (numrho - 1) / 2;
                        accum_additional[(n + 1) * (numrho + 2) + r + 1]++;
                    }
            }
    }
    else
    {
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
    }

    // stage 2. find local maximums
    if (b_view)
    {
        for (int r = 0; r < numrho; r++)
        {
            for (int n = 0; n < b_angle_roi * numangle; n++)
            {
                int base = (n + 1) * (numrho + 2) + r + 1;
                if (accum[base] > threshold &&
                    accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                    accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                    _sort_buf_left.push_back(base);
                if (accum_additional[base] > threshold &&
                    accum_additional[base] > accum_additional[base - 1] && accum_additional[base] >= accum_additional[base + 1] &&
                    accum_additional[base] > accum_additional[base - numrho - 2] && accum_additional[base] >= accum_additional[base + numrho + 2])
                    _sort_buf_right.push_back(base);
            }
            for (int n = (1. - b_angle_roi) * numangle; n < numangle; n++)
            {
                int base = (n + 1) * (numrho + 2) + r + 1;
                if (accum[base] > threshold &&
                    accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                    accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                    _sort_buf_left.push_back(base);
                if (accum_additional[base] > threshold &&
                    accum_additional[base] > accum_additional[base - 1] && accum_additional[base] >= accum_additional[base + 1] &&
                    accum_additional[base] > accum_additional[base - numrho - 2] && accum_additional[base] >= accum_additional[base + numrho + 2])
                    _sort_buf_right.push_back(base);
            }
        }
    }
    else
    {
        //horizontal lines (~0.5*pi = 90°) are excluded
        //right leaning lines are added to _sort_buf_left
        //left leaning lines are added to _sort_buf_right
        for (int r = 0; r < numrho; r++)
        {
            for (int n = 0; n < angle_roi * numangle; n++)
            {
                int base = (n + 1) * (numrho + 2) + r + 1;
                if (accum[base] > threshold &&
                    accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                    accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                    _sort_buf_left.push_back(base);
            }
            for (int n = (1. - angle_roi) * numangle; n < numangle; n++)
            {
                int base = (n + 1) * (numrho + 2) + r + 1;
                if (accum[base] > threshold &&
                    accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                    accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                    _sort_buf_right.push_back(base);
            }
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
    double range = 0.01; //range in radiants to check for last_angle, 0.01 radiants = 0.57°
    i = 0;               //counter for numbers of actual line segments
    j = 0;               //traverses the _sort_buf array
    while (i < lines_max && j < (int)_sort_buf_left.size())
    {
        LinePolar line;
        int idx = _sort_buf_left[j];
        int n = cvFloor(idx * scale) - 1;
        int r = idx - (n + 1) * (numrho + 2) - 1;
        line.rho = (r - (numrho - 1) * 0.5f) * rho;
        line.angle = static_cast<float>(min_theta) + n * theta;
        //std::cout << "i: " << i << ", last: " << last_angle << ", lineangle: " << line.angle << std::endl;
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
            std::cout << "continue" << std::endl;
            ++j;
            continue; //current line is too similar (parallel) to previous line -> skip it
        }
        last_angle = line.angle;
        left_lines.push_back(Vec2f(line.rho, line.angle));
        ++i;
        ++j;
    }
    i = 0;
    j = 0;
    while (i < lines_max && j < (int)_sort_buf_right.size())
    {
        LinePolar line;
        int idx = _sort_buf_right[j];
        int n = cvFloor(idx * scale) - 1;
        int r = idx - (n + 1) * (numrho + 2) - 1;
        line.rho = (r - (numrho - 1) * 0.5f) * rho;
        line.angle = static_cast<float>(min_theta) + n * theta;
        //std::cout << "i: " << i << ", last: " << last_angle << ", lineangle: " << line.angle << std::endl;
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
            std::cout << "continue" << std::endl;
            ++j;
            continue; //current line is too similar (parallel) to previous line -> skip it
        }
        last_angle = line.angle;
        right_lines.push_back(Vec2f(line.rho, line.angle));
        ++i;
        ++j;
    }
}

void multiple_windows_search(Mat &input_img, const int num_windows, const int width, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    multiple_windows_search_(input_img, num_windows, width, left_points, true);
    multiple_windows_search_(input_img, num_windows, width, right_points, false);
}

void partitioned_hough(const Mat &img, const int *part_coords, const int num_part, const int num_lines, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines, const bool b_view)
{
    left_lines.clear();
    right_lines.clear();
    std::vector<Vec2f> left_lines_tmp;
    std::vector<Vec2f> right_lines_tmp;
    for (int i = 0; i < num_part; ++i)
    {
        HoughLinesCustom(img, 1., CV_PI / 180., 10, left_lines_tmp, right_lines_tmp, num_lines, part_coords[i], part_coords[i + 1], b_view);
        left_lines.insert(left_lines.end(), left_lines_tmp.begin(), left_lines_tmp.end());
        right_lines.insert(right_lines.end(), right_lines_tmp.begin(), right_lines_tmp.end());
        std::cout << "part left size: " << left_lines_tmp.size() << ", part right size: " << right_lines_tmp.size() << ", part-coords i+1: " << part_coords[i+1] << std::endl;
        assert(left_lines_tmp.size() == (unsigned int) num_lines);
        assert(right_lines_tmp.size() == (unsigned int) num_lines);
        left_lines_tmp.clear();
        right_lines_tmp.clear();
    }
}

//TODO change Point to Point2f and test
void poly_reg(const std::vector<Point2f> &left_points, const std::vector<Point2f> &right_points, std::vector<double> &left_coeff, std::vector<double> &right_coeff, const int order)
{
    poly_reg_(left_points, left_coeff, order);
    poly_reg_(right_points, right_coeff, order);
}

void print_result(const Mat &image, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order, const String dir, const String file)
{
    //new red image
    assert(left_coeff.size() == right_coeff.size() == order);
    Mat result (image.rows, image.cols, CV_8UC3, Scalar(0,0,255));
    int start = 0, end = 0;

    for(int r = 0; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            start += std::pow(r, c) * left_coeff[c];
            end += std::pow(r, c) * right_coeff[c];
        }
        if(start <= end)
        {
            line(result, Point(start,r), Point(end,r), Scalar(255,0,255));
        }
        start = 0;
        end = 0;
    }
    imwrite(dir + file, result);
}

void show_image(String image_name, Mat &image, bool wait)
{
    namedWindow(image_name, WINDOW_AUTOSIZE);
    imshow(image_name, image);
    if (wait)
        waitKey(0);
}

void sub_partition(int start, int end, int number, bool equidistant, int *coords)
{
    assert(end > start);
    double factor = 1.;
    double size = 1.;
    if (!equidistant)
    {
        factor = 1. / (number + 2);
        size = 3;
    }
    int domain_length = (end - start) / number;

    coords[0] = start;
    for (int i = 1; i < number + 1; ++i)
    {
        coords[i] = coords[i - 1] + size * factor * domain_length;
        if (!equidistant)
            size += 2;
    }
    //coords[number] = end;
}

void v_roi(Mat &img, const double &start)
{
    rectangle(img, Point(0, 0), Point(img.cols, img.rows * start), Scalar(0), CV_FILLED);
}

void window_search(const Mat &img, const int *input_points, const int &window_width, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{

    left_points.clear();
    right_points.clear();
    left_points.insert(left_points.begin(), 3, Point2f(-1, -1));
    right_points.insert(right_points.begin(), 3, Point2f(-1, -1));
    const uchar *image = img.ptr();
    int low = 0.1 * img.rows, mid = 0.5 * img.rows, up = 0.9 * img.rows;
    const int offset = 50;
    assert((low - offset >= 0) && (up + offset < img.rows));

    for (int r = -offset; r < offset; ++r)
    {
        for (int c = -window_width; c < window_width; ++c)
        {
            if (image[(low + r) * img.cols + input_points[0] + c] == 255 && left_points[2].x == -1)
                left_points[2] = Point2f(input_points[0] + c, (low + r));
            if (image[(low + r) * img.cols + input_points[1] + c] == 255 && right_points[2].x == -1)
                right_points[2] = Point2f(input_points[1] + c, (low + r));
            if (image[(mid + r) * img.cols + input_points[0] + c] == 255 && left_points[1].x == -1)
                left_points[1] = Point2f(input_points[0] + c, (mid + r));
            if (image[(mid + r) * img.cols + input_points[1] + c] == 255 && right_points[1].x == -1)
                right_points[1] = Point2f(input_points[1] + c, (mid + r));
            if (image[(up + r) * img.cols + input_points[0] + c] == 255 && left_points[0].x == -1)
                left_points[0] = Point2f(input_points[0] + c, (up + r));
            if (image[(up + r) * img.cols + input_points[1] + c] == 255 && right_points[0].x == -1)
                right_points[0] = Point2f(input_points[1] + c, (up + r));
        }
    }
}