#include "fitting.hpp"

//#############################################################################################
//######################################### STATIC FCTS #######################################
//#############################################################################################

//deprecated
static void draw_curve_(Mat &image, const double roi, const std::vector<Point> &points)
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
#ifndef NDEBUG
    std::cout << "coeff for draw curve: ";
    for (unsigned int i = 0; i < points.size(); ++i)
        std::cout << sol[i] << ", " << std::endl;
#endif
    for (int r = roi * image.rows; r < image.rows; ++r)
        img[r * image.cols + static_cast<int>((sol[0] * r * r + sol[1] * r + sol[2]))] = 128;
}

static void draw_poly_(Mat &image, const double roi, const std::vector<double> &coeff, const int order)
{
    assert(coeff.size() == (unsigned int)order + 1);
    double column = 0.;
    for (int r = roi * image.rows; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            column += std::pow(r, c) * coeff[c];
        }
        //draw with a width of 5 pixels
        image.at<uchar>(r, static_cast<int>(column - 2)) = 128;
        image.at<uchar>(r, static_cast<int>(column - 1)) = 128;
        image.at<uchar>(r, static_cast<int>(column)) = 128;
        image.at<uchar>(r, static_cast<int>(column + 1)) = 128;
        image.at<uchar>(r, static_cast<int>(column + 2)) = 128;
        column = 0.;
    }
}

static void poly_reg_(const std::vector<Point2f> &points, std::vector<double> &coeff, const int order)
{
    assert(points.size() >= (unsigned int)order + 1);
    coeff.clear();
    const int num_points = points.size();
    Mat lhs = Mat_<double>(num_points, order + 1);
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

//deprecated
void draw_curve(Mat &image, const double roi, const std::vector<Point> &left_points, const std::vector<Point> &right_points)
{
    draw_curve_(image, roi, left_points);
    draw_curve_(image, roi, right_points);
}

void draw_poly(Mat &image, const double roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order)
{
    draw_poly_(image, roi, left_coeff, order);
    draw_poly_(image, roi, right_coeff, order);
}

void poly_reg(const std::vector<Point2f> &left_points, const std::vector<Point2f> &right_points, std::vector<double> &left_coeff, std::vector<double> &right_coeff, const int order)
{
    poly_reg_(left_points, left_coeff, order);
    poly_reg_(right_points, right_coeff, order);
}

int store_result(const Mat &image, const double &roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order, const String dir, const String file)
{
    //new red image
    assert(left_coeff.size() == right_coeff.size() && left_coeff.size() == 1u + order);
    if (left_coeff.size() != right_coeff.size() || right_coeff.size() != 1u + order)
        return MAPRA_ERROR;
    Mat result(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 255));
    int start = 0, end = 0;

    for (int r = roi * image.rows; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            start += std::pow(r, c) * left_coeff[c];
            end += std::pow(r, c) * right_coeff[c];
        }
        if (start <= end)
        {
            line(result, Point(start, r), Point(end, r), Scalar(255, 0, 255));
        }
        start = 0;
        end = 0;
    }
#ifndef NDEBUG
    std::cout << "File written to path: " << dir + file << std::endl;
#endif
    imwrite(dir + file, result);
    return MAPRA_SUCCESS;
}