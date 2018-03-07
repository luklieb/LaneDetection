#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

struct calib_data
{
    const Mat *img;
    double *const array;
};

/**
 * Opens two windows, one showing sliders to adjust the trapezoids parameters needed for the warpPerspective() call,
 * the other one shows the bird-view according to the set parameters
 * @param img Image that the trapezoids are drawn on
 * @param offset_mid B_VIEW initial parameter
 * @param offset B_VIEW initial parameter
 * @param offset2 B_VIEW initial parameter
 * @param height B_VIEW initial parameter
 */
void b_view_calibration(Mat *img, double offset_mid = 0., double offset = 0., double offset2 = 0., double height = 0.);