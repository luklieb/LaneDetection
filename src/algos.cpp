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
 * Checks the return codes of two algorthims (left and right lane call)
 * and returns the appropriate code
 */
static int inline check_codes(int code1, int code2)
{
    if (code1 == LANEDET_ERROR || code2 == LANEDET_ERROR)
        return LANEDET_ERROR;
    if (code1 == LANEDET_WARNING || code2 == LANEDET_WARNING)
        return LANEDET_WARNING;
    return LANEDET_SUCCESS;
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

static int alm(std::vector<Point2f> &points, const int num_part, const int num_lines)
{
    assert(points.size() % 2 == 0);

    std::vector<std::vector<Point2f>> possible_points(num_lines);
    double error_sum[num_lines];
    double error_tmp;
    int line_tmp;
    Point2f start_pt;
    Point2f end_pt;

    //there are num_lines in each partition
    //iterate over almost all line segment combinations to determine an error norm
    //(skip the ones where the new error norm is already larger than the error_tmp nrom)
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
    //choose the line combination (over the num_part partitions) with the lowest error norm
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
    //store the best (lowest error norm) line combinations in the original vector
    points.clear();
    points.insert(points.begin(), possible_points[index_tmp].begin(), possible_points[index_tmp].end());

    assert(points.size() == 2u * num_part);
    if (points.size() != 2u * num_part)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    return LANEDET_SUCCESS;
}

/**
 * Converts the polar coordiantes from a Hough Transform (lines) to points according to their start/end of each partition
 * @param lines The polar coordinates returned from a Hough Transform
 * @param num_lines The number of lines of each side of each partition
 * @param num_part The number of partitions
 * @param coords_part Coordinates of the partitions
 * @param points Holds the converted Points 
 * @return Returns either LANEDET_SUCCESS or LANEDET_WARNING
 * @note Storage order for both [left/right]_points: points belonging to same line are stored consecutively:
 *      Point_0(line_0), Point_1(line_0), Point_2(line_1), Point_3(line_1), ..., 
 *      Point_num_lines*2-1(line_num_lines-1), ..., Point_num_lines*num_part*2-1(line_num_lines*num_part-1) 
 * @note calls get_points(std::vector<Vec2f> &, const int, const int, const int *, std::vector <Point2f> &) once for each side
 */
static int get_points(const std::vector<Vec2f> &lines, const int num_lines, const int num_part, const int *coords_part, std::vector<Point2f> &points)
{
#ifndef NDEBUG
    std::cout << "get_points: " << num_lines << ", " << num_part << std::endl;
#endif
    assert(lines.size() == 1u * num_lines * num_part);
    if (lines.size() != 1u * num_lines * num_part)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    points.clear();
    int i = 0;
    int j = 0;
    Point2f pt1, pt2;
    //iterate over all lines
    //there are always num_lines in each num_part
    //transform one line in polar coordinates (2 variables (roh, phi)) to
    //one start point and one end point in cartesian coordiantes (2 variables (x,y) each)
    for (auto p : lines)
    {
        if (i >= num_part)
            break;
        float rho = p[0], theta = p[1];
        double a = cos(theta), b = sin(theta), a_inv = 1. / a;
        pt1.y = coords_part[i];
        pt1.x = (rho - pt1.y * b) * a_inv;
        pt2.y = coords_part[i + 1];
        pt2.x = (rho - pt2.y * b) * a_inv;
        points.push_back(pt1);
        points.push_back(pt2);
        ++j;
        //after num_lines continue with next partition
        if (j % num_lines == 0)
            ++i;
    }
    assert(points.size() == 2u * num_part * num_lines);
    if (points.size() != 2u * num_part * num_lines)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    return LANEDET_SUCCESS;
}

static int get_points(const std::vector<Vec2f> &left_lines, const std::vector<Vec2f> &right_lines, const int num_lines,
                      const int num_part, const int *coords_part, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    int code1 = get_points(left_lines, num_lines, num_part, coords_part, left_points);
    int code2 = get_points(right_lines, num_lines, num_part, coords_part, right_points);
    return check_codes(code1, code2);
}

/**
 * Returns the two horizontal (x-coordinates) points (one for right/left half) with the maximum according to the histogram of the ROI
 * @param img is the binary input image
 * @param roi Vertical starting point in percent of the region of interest
 * @param points holds the 2 x-coordinates according to the two max values of the histogram
 * @return Returns either LANEDET_SUCCESS or LANEDET_WARNING
 * @note reduce() function could be optimized with an additional roi constraint
 */
static int h_histogram(const Mat &img, const double roi, int *x_points)
{
    std::vector<int> histo;
    //"sums up" along y-axis for each column -> returns a "row vector"
    //new Mat is the only the roi part of img
    reduce(Mat(img, Range(roi * img.rows, img.rows)), histo, 0, CV_REDUCE_SUM);
    int m1 = 0;
    int m2 = 0;
    x_points[0] = -1;
    x_points[1] = -1;
    //consider two halfes of image independently
    //find "peak" in histogram for each side and store it in x_points array
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
    assert(x_points[0] != -1 && x_points[1] != -1 && x_points[0] <= x_points[1]);
    if (x_points[0] == -1 || x_points[1] == -1 || x_points[0] > x_points[1])
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    return LANEDET_SUCCESS;
}

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
 * @return Returns either LANEDET_SUCCESS, LANEDET_WARNING or LANEDET_ERROR
 * @note be aware that if the current line is similar (parrallel) to the previous line it gets ignored
 *       -> might cause problems with "bird eye view" (because then both road lines are parallel)
 * @note: add further failsafe (one more bool to check wheter birdsview or not and then
 * add restrictions in function to restrict e.g. the angle of the second line, etc.)
 */
static int hough_lines_custom(const Mat &img, const float rho, const float theta, const int threshold,
                              std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines,
                              const int lines_max, const int roi_start, const int roi_end, const bool b_view, const double min_theta = 0, const double max_theta = CV_PI)
{

    //Additional parameters, that can be changed, but make everything way more complicated
    //Automaticly tuning them is too much work for this scope
    //Excludes relativley horizontal lines (only accepts "steep" angles between [0, excl_hor*pi] and [(1-excl_hor)*pi, pi])
    const double excl_hor = 0.4;
    //Excludes completely vertical lines from normal-view transformation
    //Normal-View searches for angles in range [excl_vert*pi, excl_hor*pi] and [(1-excl_hor)*pi, (1-excl_vert)*pi]
    double excl_vert = 0.03;
    //Similar to excl_hor, but now only for b_view == true
    //Since the searched for lanes in the Birdview perspective are steeper, b_excl_hor < excl_hor
    const double b_excl_hor = 0.1;
    //Only needed if b_view == true. Relative width of one side to look for lines for the respective left or right lane.
    //Left lane is searched in [0, b_roi_widht*width]; Right lanes is searched in [(1-b_roi_width)*width, width].
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
        return LANEDET_ERROR;
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
    //In normal-view we can easily differentiate between left and right lane by their respective line angles,
    //    because of the perspective distortion the lanes appear to meet on the horizon ->
    //    Angle of left lane is ~[0, 0.5*PI] in parameter space; angle of right lane is ~[0.75*PI, PI]
    //In bird-view it is hard to differentiate between left and right lane, because now the lanes should be more or less
    //    parallel to each other -> split image in the middle and look for left/right lanes in the respective half
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
        //left side of image for left lane in accum
        for (i = roi_start; i < roi_end; i++)
            for (j = 0; j < b_roi_width * width; j++)
            {
                if (image[i * step + j] != 0)
                    for (int n = 0; n < numangle; n++)
                    {
                        //radius = shortest (perpendicular) distance to line
                        int r = cvRound(j * tabCos[n] + i * tabSin[n]);
                        r += (numrho - 1) / 2;
                        accum[(n + 1) * (numrho + 2) + r + 1]++;
                    }
            }
        //right side of image for right lane in accum_additional
        for (i = roi_start; i < roi_end; i++)
            for (j = (1. - b_roi_width) * width; j < width; j++)
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
    else //normal view
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
            for (int n = 0; n < b_excl_hor * numangle; n++)
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
            for (int n = (1. - b_excl_hor) * numangle; n < numangle; n++)
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
    else //normal view
    {
        //more or less horizontal lines (~0.5*pi = 90°) are excluded
        //right leaning lines are added to _sort_buf_left
        //left leaning lines are added to _sort_buf_right
        for (int r = 0; r < numrho; r++)
        {
            for (int n = excl_vert * numangle; n < excl_hor * numangle; n++)
            {
                int base = (n + 1) * (numrho + 2) + r + 1;
                if (accum[base] > threshold &&
                    accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
                    accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2])
                    _sort_buf_left.push_back(base);
            }
            for (int n = (1. - excl_hor) * numangle; n < (1. - excl_vert) * numangle; n++)
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
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
#ifndef NDEBUG
            std::cout << "hough continue" << std::endl;
#endif
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
        if (i > 0 && line.angle >= last_angle - range && line.angle <= last_angle + range)
        {
#ifndef NDEBUG
            std::cout << "hough continue" << std::endl;
#endif
            ++j;
            continue; //current line is too similar (parallel) to previous line -> skip it
        }
        last_angle = line.angle;
        right_lines.push_back(Vec2f(line.rho, line.angle));
        ++i;
        ++j;
    }
    assert(left_lines.size() == (unsigned int)lines_max && right_lines.size() == (unsigned int)lines_max);
    if (left_lines.size() != (unsigned int)lines_max || right_lines.size() != (unsigned int)lines_max)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    return LANEDET_SUCCESS;
}

/**
 * Converts the points-pairs making up lines returned from alm() or partitoned_hough() with num_lines = 1
 * into single points (in order to compute the coefficients)
 * It computes the x-cooordinate average of two points with the same y-coordinate (two adjacent lines)
 * Before each line has its own start and end point, after the call each end point of a line segment 
 * is the start point of the next line segment
 * @param left_points Holds the points-pairs for the left lane
 * @param left_points Holds the points-pairs for the right lane
 * @return Returns either LANEDET_WARNING or LANEDET_SUCESS
 * @note Calls pair_conversion(std::vector<Point2f>) once for each side
 */
static int pair_conversion(std::vector<Point2f> &points)
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
        assert(cpy[i].y == cpy[i + 1].y);
        if (cpy[i].y != cpy[i + 1].y)
        {
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }
        points.push_back(Point2f(0.5 * (cpy[i].x + cpy[i + 1].x), cpy[i].y));
    }
    //last point can stay
    points.push_back(cpy[size - 1]);

    assert(points.size() == 0.5 * cpy.size() + 1.);
    if (points.size() != 0.5 * cpy.size() + 1.)
        return LANEDET_WARNING;
    return LANEDET_SUCCESS;
}

static int pair_conversion(std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    int code1 = pair_conversion(left_points);
    int code2 = pair_conversion(right_points);
    return check_codes(code1, code2);
}

/**
 * Applies the Hough Transformation on different sub-regions 
 * @param img The image for the Hough Transformation
 * @param part_coords Holds the num_part+1 coordinates of the partitions
 * @param num_part Number of sub-domains/partitions
 * @param num_lines Number of lines per partition per side 
 * @param left_lines Vector holding the polar coordinates of the detected lines on the left side (lane)
 * @param right_lines Vector holding the polar coordinates of the detected lines on the right side (lane)
 * @param b_view If true, Hough Transformation is slightly optimized for the bird-view-perspective
 * @return Returns either LANEDET_SUCCESS, LANEDET_WARNING
 * @note [left/right]_lines stores at the beginning num_lines lines of the first partition,
 * 		at the end num_lines lines of the last partition
 */
static int partitioned_hough(const Mat &img, const int *part_coords, const int num_part, const int num_lines, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines, const bool b_view)
{
    left_lines.clear();
    right_lines.clear();
    std::vector<Vec2f> left_lines_tmp;
    std::vector<Vec2f> right_lines_tmp;
    int code;
    for (int i = 0; i < num_part; ++i)
    {
        //call hough for with the right start and end coordinats
        code = hough_lines_custom(img, 1., CV_PI / 180., 10, left_lines_tmp, right_lines_tmp, num_lines, part_coords[i], part_coords[i + 1], b_view);
        if (code != LANEDET_SUCCESS)
            return code;

        //add the found lines from each partiton to vector with all lines
        left_lines.insert(left_lines.end(), left_lines_tmp.begin(), left_lines_tmp.end());
        right_lines.insert(right_lines.end(), right_lines_tmp.begin(), right_lines_tmp.end());

#ifndef NDEBUG
        std::cout << "part left size: " << left_lines_tmp.size() << ", part right size: " << right_lines_tmp.size() << ", part-coords i+1: " << part_coords[i + 1] << std::endl;
#endif
        assert(left_lines_tmp.size() == (unsigned int)num_lines && right_lines_tmp.size() == (unsigned int)num_lines);
        if (left_lines_tmp.size() != (unsigned int)num_lines || right_lines_tmp.size() != (unsigned int)num_lines)
        {
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }
        
        left_lines_tmp.clear();
        right_lines_tmp.clear();
    }
    return LANEDET_SUCCESS;
}

static int sliding_windows_search(Mat &img, const double roi, const int num_windows, const int width, std::vector<Point2f> &points, const bool left)
{
    int upper_histo[2];
    points.clear();
    //has to be of type Point. Stores the idizes of white (non-black) pixels
    std::vector<Point> non_zero;
    //starting points for first windows (left and right half)
    int code = h_histogram(img, roi, upper_histo);
    if (code != LANEDET_SUCCESS)
        return code;
#ifndef NDEBUG
    std::cout << "histo done: " << upper_histo[0] << ", " << upper_histo[1] << std::endl;
#endif

    //offsets from center points of window (starting points)
    const int height = img.rows / num_windows - 1;
    const int y_offset = 0.5 * height + 1;
    const int x_offset = 0.5 * width;
    int x_tmp = 0;
    int x;
    if (left)
    {
        x = upper_histo[0];
        if (x - x_offset < 0)
            x = x_offset;
    }
    else
    {
        x = upper_histo[1];
        if (x + x_offset >= img.cols)
            x = img.cols - 1 - x_offset;
    }

    assert(x - x_offset >= 0);
    if (x - x_offset < 0)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    assert(x - x_offset + width < img.cols);
    if (x - x_offset + width >= img.cols)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }

    int y_tmp = 0;
    int y = img.rows - y_offset;

    //iterate over the windows (from higher y coordinates towards lower y coordinates)
    for (int i = 0; i < num_windows; ++i)
    {
#ifndef NDEBUG
        std::cout << "nonzero"
                  << " x: " << x - x_offset << ", y: " << y - y_offset << std::endl;
        std::cout << "image c: " << img.cols << ", r: " << img.rows
                  << ", width: " << width << ", height: " << height << std::endl;
#endif

        try
        {
            //find all indizes of white (non black) pixels --> so we can compute the mean of them later
            findNonZero(Mat(img, Rect(x - x_offset, y - y_offset, width, height)), non_zero);
        }
        catch (cv::Exception &e)
        {
            //if sliding windows move out of the picture
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }

#ifndef NDEBUG
        std::cout << "nonzero done in sliding window: " << i << std::endl;
#endif

        x_tmp = 0;
        y_tmp = 0;

        //if non-black pixels are found
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
            else //if not, move up in a straight line
            {
                x_tmp = x;
                y_tmp = y;
            }
#ifndef NDEBUG
            std::cout << "multiple_window no white pixels found" << std::endl;
#endif
        }

        points.push_back(Point2f(x_tmp, y_tmp));

        //move search window one step down
        y -= height;
        x = x_tmp;
        //if new search coordinates are not in the image, they are probably not part of a road lane
        assert(x + x_offset < img.cols);
        if (x + x_offset >= img.cols)
        {
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }
        assert(x - x_offset >= 0);
        if (x - x_offset < 0)
        {
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }

        non_zero.clear();
    }
    assert(points.size() == (unsigned int)num_windows);
    if (points.size() != (unsigned int)num_windows)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }
    return LANEDET_SUCCESS;
}

/**
 * Returns the (x or y) coordinates of the sub-domains
 * @param start of the whole domain to be partitioned
 * @param end of the whole domain to be partitioned
 * @param euqidistant whether or not sub-domain borders should be euqidistant
 * @param number of sub-domains
 * @param coords array of length number+1, returns the coordinates of the sub-domain borders (including start and end)
 */
static void sub_partition(const int start, const int end, const int number, const bool equidistant, int *coords)
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

/**
 * ### Deprecated ###
 * Takes a binary image and fills it with black (=0) except for the vertical region of interest (roi) (-> along the y axis)
 * @param img is the image to be modified and returned
 * @param start is the vertical start where the roi is starting in percent [0;1]
 */
static inline void v_roi(Mat &img, const double start)
{
    rectangle(img, Point(0, 0), Point(img.cols, img.rows * start), Scalar(0), CV_FILLED);
}

//#############################################################################################
//######################################### INTERFACE #########################################
//#############################################################################################

int alm(const Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part, const int num_lines,
        const bool b_view, const double roi)
{
    int coords_part[num_part + 1];
    //supartition for roi in image
    sub_partition(roi * img.rows, img.rows, num_part, true, coords_part);
    std::vector<Vec2f> left_lines;
    std::vector<Vec2f> right_lines;
    //call hough for each partition and store multiple found lanes per partitions
    int code = partitioned_hough(img, coords_part, num_part, num_lines, left_lines, right_lines, b_view);
    if (code != LANEDET_SUCCESS)
        return code;
    code = get_points(left_lines, right_lines, num_lines, num_part, coords_part, left_points, right_points);
    if (code != LANEDET_SUCCESS)
        return code;

    //take the multiple found lanes and find the "best" combination of them
    int code1 = alm(left_points, num_part, num_lines);
    int code2 = alm(right_points, num_part, num_lines);

    //compute the mean x-coordinate of the two line end/starting points on one partition boundary
    code = pair_conversion(left_points, right_points);
    if (code != LANEDET_SUCCESS)
        return code;
    return check_codes(code1, code2);
}

int hough(Mat &img, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, const int num_part,
          const bool b_view, const double roi)
{
    //Get coordinates of individual partitions
    int coords_part[num_part + 1];
    sub_partition(roi * img.rows, img.rows, num_part, true, coords_part);
    std::vector<Vec2f> left_lines;
    std::vector<Vec2f> right_lines;
    //Conduct houghtransformations on the partitions
    //Only 1 line per partition per side
    int code = partitioned_hough(img, coords_part, num_part, 1, left_lines, right_lines, b_view);
#ifndef NDEBUG
    std::cout << left_lines.size() << ", " << right_lines.size() << std::endl;
#endif
    if (code != LANEDET_SUCCESS)
        return code;
    //Transform line-coordinates from parameter space to coordinates space
    //Get the intersection points of partition borders and lines
    code = get_points(left_lines, right_lines, 1, num_part, coords_part, left_points, right_points);

    assert(left_points.size() == 2u * num_part && right_points.size() == 2u * num_part);
    if (left_points.size() != 2u * num_part || right_points.size() != 2u * num_part)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }

#ifndef NDEBUG
    for (auto p = left_points.begin(); p != left_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 3);
    for (auto p = right_points.begin(); p != right_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 3);
    show_image("houg_part", img, true);
#endif

    //Compute and return the mean of the two points for each side with same y-coordinate (on partition boundary)
    pair_conversion(left_points, right_points);

#ifndef NDEBUG
    for (auto p = left_points.begin(); p != left_points.end(); p += 1)
        line(img, *p, *(p), Scalar(255), 10);
    for (auto p = right_points.begin(); p != right_points.end(); p += 1)
        line(img, *p, *(p), Scalar(255), 10);
    show_image("houg_part after conversion", img, true);
#endif

    if (code != LANEDET_SUCCESS)
        return code;
    return LANEDET_SUCCESS;
}

int random_search(Mat &img, const int num_lines, const double roi, const int num_part, const bool b_view, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    //0% overlap for each half
    const double image_split = 0.5;
    //range to search around current line for white pixels
    const unsigned int offset_x = 5;
    //standard deviation for normal distribution
    const double sigma = 70.;
    //mean for left side distribution (closer to middle of picture)
    const double m_l = 0.75 * img.cols * 0.55;
    //mean for right side distribution (closer to middle of picture)
    const double m_r = img.cols - m_l - 1.;
    //maximum amount of pixels, that e_l-s_l respectively -(e_r-s_r) can differ
    //this excludes lines with wrong slope
    //-> most of left lines are right leaning, most of right lines are left leaning (same as road lanes)
    const int pixel_diff = 50;

    std::default_random_engine generator;
    std::normal_distribution<double> dist_left(m_l, sigma);
    std::normal_distribution<double> dist_right(m_r, sigma);
    //coordinates of partition borders
    int coords_part[num_part + 1];
    sub_partition(roi * img.rows, img.rows, num_part, true, coords_part);
    //[start/end] [left/right] x values of points
    int s_l, e_l, s_r, e_r;
    //store temporarily the best x-[start/end]-[left/right]-points
    int s_l_best = 0, e_l_best = 0, s_r_best = 0, e_r_best = 0;
    //score of line quality (sum of white pixels along the current line)
    int score_l = 0, score_r = 0;
    //store temporarily the best score ->
    int score_l_best = 0, score_r_best = 0;
    //variables for line parameter calculation
    double slope_l, slope_r;
    double height_inv;
    double height;
    //x-coordinates of the current line
    int x_l_curr, x_r_curr;

    //for each partition
    for (int part = 0; part < num_part; ++part)
    {
        //height of current partition
        height = (coords_part[part + 1] - coords_part[part]);
        height_inv = 1. / height;
        assert(height_inv >= 0.);

        //create num_lines on both sides and calculate for the current line the quality
        //(-> move along the line an count white pixels)
        //if current line for one side is better than the former best, temporarily store the configuration
        for (int l = 0; l < num_lines;)
        {
            s_l = dist_left(generator);
            e_l = dist_left(generator);
            s_r = dist_right(generator);
            e_r = dist_right(generator);
            if(e_l - s_l > pixel_diff || e_r -s_r < -pixel_diff)
                continue;
            ++l;
#ifndef NDEBUG
            line(img, Point2f(s_l, coords_part[part]), Point2f(e_l, coords_part[part + 1]), Scalar(128));
            line(img, Point2f(s_r, coords_part[part]), Point2f(e_r, coords_part[part + 1]), Scalar(128));
#endif
            //slope of lines like this: x = slope*y + t
            slope_l = height_inv * (e_l - s_l);
            slope_r = height_inv * (e_r - s_r);

            //calc quality scores for both lines
            for (int y = 0; y < height; ++y)
            {
                x_l_curr = y * slope_l + s_l;
                x_r_curr = y * slope_r + s_r;

                if (x_l_curr < 0 || x_l_curr > img.cols || x_r_curr < 0 || x_r_curr > img.cols || x_l_curr > image_split*img.cols || x_r_curr < (1.-image_split)*img.cols)
                {
                    //normal distribution might give us points that are outside of the image
                    //-> skip them
                    continue;
                }

                //search for 2*offset_x pixels around the line for white pixels
                //(only if pixel is in the image [0, img.cols])
                for (int x_offset = -offset_x; x_offset <= (int)offset_x; ++x_offset)
                {
                    if (x_l_curr + x_offset > 0 && x_l_curr + x_offset < img.cols && img.at<uchar>(y + coords_part[part], x_l_curr + x_offset) >= 250)
                        ++score_l;
                    if (x_r_curr + x_offset > 0 && x_r_curr + x_offset < img.cols && img.at<uchar>(y + coords_part[part], x_r_curr + x_offset) >= 250)
                        ++score_r;
                }
            }
            //store current configuration for both sides
            //if it is better than the all time best
            if (score_l > score_l_best)
            {
                score_l_best = score_l;
                s_l_best = s_l;
                e_l_best = e_l;
            }
            if (score_r > score_r_best)
            {
                score_r_best = score_r;
                s_r_best = s_r;
                e_r_best = e_r;
            }
            score_l = 0;
            score_r = 0;
        }
        left_points.push_back(Point2f(s_l_best, coords_part[part]));
        left_points.push_back(Point2f(e_l_best, coords_part[part + 1]));
        right_points.push_back(Point2f(s_r_best, coords_part[part]));
        right_points.push_back(Point2f(e_r_best, coords_part[part + 1]));
        score_l_best = 0;
        score_r_best = 0;
    }
    if (left_points.size() != num_part * 2u || right_points.size() != num_part * 2u)
    {
        std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }

#ifndef NDEBUG
    show_image("all random", img, true);
    for (auto p = left_points.begin(); p != left_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 4);
    for (auto p = right_points.begin(); p != right_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 4);
    std::cout << "num points after random(): " << left_points.size() << ", right: " << right_points.size() << std::endl;
    show_image("random finished", img, true);
#endif

    //Compute and return the mean of the two points for each side with same y-coordinate (on partition boundary)
    pair_conversion(left_points, right_points);

    return LANEDET_SUCCESS;
}

int sliding_windows_search(Mat &img, const double roi, const int num_windows, const int width, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    int code1 = sliding_windows_search(img, roi, num_windows, width, left_points, true);
    int code2 = sliding_windows_search(img, roi, num_windows, width, right_points, false);
    return check_codes(code1, code2);
}

int fixed_window_search(const Mat &img, const int window_width, const double roi, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{

    int input_points[2];
    h_histogram(img, roi, input_points);
#ifndef NDEBUG
    std::cout << "input points: " << input_points[0] << ", " << input_points[1] << std::endl;
#endif
    left_points.clear();
    right_points.clear();
    Point2f check_point(-1, -1);
    left_points.insert(left_points.begin(), 3, check_point);
    right_points.insert(right_points.begin(), 3, check_point);
    const uchar *image = img.ptr();
    //3 search regions (aka windows)
    const int low = 0.1 * (1. - roi) * img.rows + roi * img.rows;
    const int mid = 0.5 * (1. - roi) * img.rows + roi * img.rows;
    const int up = 0.9 * (1. - roi) * img.rows + roi * img.rows;
    //half amount of pixels to search in vertical direction
    const int offset = 0.1 * (1. - roi) * img.rows - 1;
    assert((low - offset >= 0) && (up + offset < img.rows));
    if ((low - offset < 0) || (up + offset >= img.rows))
        return LANEDET_ERROR;

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
#ifndef NDEBUG
    std::cout << "multiple window search points found:" << std::endl;
    for (auto p : left_points)
    {
        line(img, p, p, Scalar(255), 9);
        std::cout << p << std::endl;
    }
    for (auto p : right_points)
    {
        line(img, p, p, Scalar(255), 9);
        std::cout << p << std::endl;
    }
    show_image("window_s", img, true);
#endif

    for (int i = 0; i < 3; ++i)
    {
        assert(left_points[i] != check_point && right_points[i] != check_point);
        if (left_points[i] == check_point || right_points[i] == check_point)
        {
            std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
            return LANEDET_WARNING;
        }
    }
    return LANEDET_SUCCESS;
}