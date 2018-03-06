#include "fitting.hpp"

//#############################################################################################
//######################################### STATIC FCTS #######################################
//#############################################################################################

//deprecated
static void draw_curve(Mat &image, const double roi, const std::vector<Point> &points)
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

static void draw_poly(Mat &image, const double roi, const std::vector<double> &coeff, const int order, const Mat &b_inv)
{
    assert(coeff.size() == (unsigned int)order + 1);
    double column = 0.;
    std::vector<Point2f> pts;
    std::vector<Point2f> pts_trans;
    for (int r = roi * image.rows; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            column += std::pow(r, c) * coeff[c];
        }
        pts.push_back(Point2f(column, r));
        column = 0.;
    }
    if (b_inv.rows == 3 && b_inv.cols == 3)
    {
        perspectiveTransform(pts, pts_trans, b_inv);
        for (auto p = pts_trans.begin(); p != pts_trans.end() - 1; ++p)
            line(image, *p, *(p + 1), Scalar(255), 5);
    }
    else
    {
        for (auto p = pts.begin(); p != pts.end() - 1; ++p)
            line(image, *p, *(p + 1), Scalar(255), 5);
    }
}

static void poly_reg(const std::vector<Point2f> &points, std::vector<double> &coeff, const int order)
{
    assert(points.size() >= (unsigned int)order + 1);
    coeff.clear();
    const int num_points = points.size();
    Mat lhs = Mat_<double>(num_points, order + 1);
    Mat rhs = Mat(num_points, 1, CV_64F);
    Mat solution = Mat_<double>();

    //constructs simple least squares regression matrix (not Vandermonde matrix)
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
    for (int i = 0; i <= order; ++i)
        coeff.push_back(sol[i]);
}

//#############################################################################################
//######################################### INTERFACE #########################################
//#############################################################################################

//deprecated
void draw_curve(Mat &image, const double roi, const std::vector<Point> &left_points, const std::vector<Point> &right_points)
{
    draw_curve(image, roi, left_points);
    draw_curve(image, roi, right_points);
}

void draw_poly(Mat &image, const double roi, const std::vector<double> &left_coeff, const std::vector<double> &right_coeff, const int order, const Mat &b_inv)
{
    draw_poly(image, roi, left_coeff, order, b_inv);
    draw_poly(image, roi, right_coeff, order, b_inv);
}

void poly_reg(const std::vector<Point2f> &left_points, const std::vector<Point2f> &right_points, std::vector<double> &left_coeff, std::vector<double> &right_coeff, const int order)
{
    poly_reg(left_points, left_coeff, order);
    poly_reg(right_points, right_coeff, order);
}

int store_result(const Mat &image, const double &roi, const std::vector<double> &left_coeff,
                 const std::vector<double> &right_coeff, const int order, const String dir, const String file, const Mat &b_inv)
{
    //new red image
    assert(left_coeff.size() == right_coeff.size() && left_coeff.size() == 1u + order);
    if (left_coeff.size() != right_coeff.size() || right_coeff.size() != 1u + order)
        return MAPRA_ERROR;

    Mat result(image.rows, image.cols, CV_8UC3, Scalar(0, 0, 255));
    int start = 0, end = 0;
    //if B_View is used, then we need to transform the pixels to be drawn
    std::vector<Point2f> points_l;
    std::vector<Point2f> points_l_trans;
    std::vector<Point2f> points_r;
    std::vector<Point2f> points_r_trans;

    for (int r = roi * image.rows; r < image.rows; ++r)
    {
        for (int c = 0; c <= order; ++c)
        {
            start += std::pow(r, c) * left_coeff[c];
            end += std::pow(r, c) * right_coeff[c];
        }
        if (start <= end)
        {
            points_l.push_back(Point2f(start, r));
            points_r.push_back(Point2f(end, r));
        }
        start = 0;
        end = 0;
    }
    //if a valid b_inv Mat was used as an argument
    if (b_inv.rows == 3 && b_inv.cols == 3)
    {
        //transform the found points from Bird-View to normal-view
        perspectiveTransform(points_l, points_l_trans, b_inv);
        perspectiveTransform(points_r, points_r_trans, b_inv);
        Point pts[1][4];
        int npt[1] = {4};
        assert(points_l_trans.size() == points_r_trans.size());
        for (unsigned int i = 0; i < points_l_trans.size() - 1; i += 1)
        {
            //create polygons in order to fill gaps inbetween points on each side
            pts[0][0] = points_l_trans[i];
            pts[0][1] = points_l_trans[i + 1];
            pts[0][3] = points_r_trans[i];
            pts[0][2] = points_r_trans[i + 1];
            const Point *pts_ptr[1] = {pts[0]};
            fillPoly(result, pts_ptr, npt, 1, Scalar(255, 0, 255));
        }
    }
    //we are already in normal-view
    else
    {
        for (unsigned int i = 0; i < points_l.size(); ++i)
        {
            //just connect corresponding pixels on same row with each other
            line(result, points_l[i], points_r[i], Scalar(255, 0, 255));
        }
    }

#ifndef NDEBUG
    std::cout << "File written to path: " << dir + file << std::endl;
#endif

    imwrite(dir + file, result);
    return MAPRA_SUCCESS;
}