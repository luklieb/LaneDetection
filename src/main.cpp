#include <iostream>
#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "algos.hpp"
#include "filters.hpp"
#include "fitting.hpp"
#include "helper.hpp"
#include "calibration.hpp"

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



int main(int argc, char **argv)
{
    if (argc != 4)
    {
        printf("not right amount of args\n Call like this:\
            ./mapra <input_directory_path> <input_file_name> <result_directory_path\n");
        return MAPRA_ERROR;
    }
    String input_dir = modify_dir(argv[1]);
    String input_file = argv[2];
    String result_dir = modify_dir(argv[3]);
#ifndef NDEBUG
    std::cout << input_dir << ", " << result_dir << std::endl;
#endif
    String image_location = input_dir + input_file;
    Mat image;
    Mat clone;
    Mat calibration; //used for b_view calibration
    image = imread(image_location, 1);
    clone = image.clone();
    calibration = image.clone();

    if (!image.data)
    {
        printf("No image data\n");
        return MAPRA_ERROR;
    }

    //#############################################################################################
    //######################################### Parameter File ####################################
    //#############################################################################################

    //1 = part. Hough, 2 = ALM, 3 = Sliding Window, 4 = Multiple Window
    const int ALGO = 4;            //1,2,3,4
    //Amount of partitions and lines
    int NUM_PART = 2;  //2-5
    int NUM_LINES = 5; //2-5
    //Sliding Window Constants
    int W_NUM_WINDOWS = 10; //3-10
    int W_WIDTH = 60;       //20, 40, 60, 80

    //Birdview Constants
    bool B_VIEW = true;          //true, false
    double B_OFFSET_MID = 0.04;  //const
    double B_OFFSET = 0.4;       //const
    double B_OFFSET2 = 0.05;     //const
    double B_HEIGHT = 0.56; //const
    Mat b_mat;
    Mat b_inv_mat;

    double ROI_START = 0.56; //const
    if(B_VIEW)
        ROI_START = 0.;

    //Edge detection filters
    //1 = canny, 2 = sobel_mag, 3 = sobel_par, 4 = color_thres
    //1; 2; 3; 4; 1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 1,2,3; 2,3,4; 1,3,4; 1,2,4; 1,2,3,4
    std::vector<int> FILTERS = {1,2};
    assert(FILTERS.size() >= 1);

    //Canny Parameter
    int CA_THRES; //const
    int KERNEL;       //const
    //Sobel Thresholding Parameter
    int S_MAG;
    int S_PAR_X;
    int S_PAR_Y;
    //Color Thresholding Parameter
    int C_THRES;

    //parameters tuned by hand according to 4 different scenarios, 
    //which have the largest effect on filters
    if (!B_VIEW && FILTERS.size() > 1)
    {
        CA_THRES = 40;
        KERNEL = 3;
        S_MAG = 90;
        S_PAR_X = 200;
        S_PAR_Y = 100;
        C_THRES = 150;
    }
    if (B_VIEW && FILTERS.size() > 1)
    {
        CA_THRES = 250;
        KERNEL = 5;
        S_MAG = 50;
        S_PAR_X = 15;
        S_PAR_Y = 150;
        C_THRES = 150;
    }
    if (!B_VIEW && FILTERS.size() == 1)
    {
        CA_THRES = 350;
        KERNEL = 5;
        S_MAG = 240;
        S_PAR_X = 200;
        S_PAR_Y = 100;
        C_THRES = 225;
    }
    if (B_VIEW && FILTERS.size() == 1)
    {
        CA_THRES = 350;
        KERNEL = 5;
        S_MAG = 125;
        S_PAR_X = 10;
        S_PAR_Y = 100;
        C_THRES = 210;
    }


    //Fitting Constants
    int ORDER = 2; //const
    if(ALGO == 4)
        ORDER = 2;

    //#############################################################################################
    //######################################### Program ###########################################
    //#############################################################################################

    //**********************************************************
    //****************** Preliminary Setup *********************
    //**********************************************************

    b_view_calibration(&calibration, B_OFFSET_MID, B_OFFSET, B_OFFSET2, B_HEIGHT);

    if (B_VIEW)
    {
        //both trapezoids have to be adjusted according to the size of the input image and the ROI
        //In the end both road lanes should be parallel in the Birds Eye View
        const Point2f b_p1[4] = {Point2f((0.5 - B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows),
                                 Point2f((0.5 + B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows),
                                 Point2f((0.5 + B_OFFSET_MID + B_OFFSET) * image.cols, image.rows),
                                 Point2f((0.5 - B_OFFSET_MID - B_OFFSET) * image.cols, image.rows)};
        const Point2f b_p2[4] = {Point2f((0.5 - B_OFFSET_MID - B_OFFSET2) * image.cols, 0),
                                 Point2f((0.5 + B_OFFSET_MID + B_OFFSET2) * image.cols, 0),
                                 Point2f((0.5 + B_OFFSET_MID + B_OFFSET2) * image.cols, image.rows),
                                 Point2f((0.5 - B_OFFSET_MID - B_OFFSET2) * image.cols, image.rows)};
        b_mat = getPerspectiveTransform(b_p1, b_p2);
        b_inv_mat = getPerspectiveTransform(b_p2, b_p1);
        warpPerspective(image, image, b_mat, Size(image.cols, image.rows));
    }

    //**********************************************************
    //****************** Edge Detection ************************
    //**********************************************************

    multi_filter(image, FILTERS, CA_THRES, KERNEL, S_MAG, S_PAR_X, S_PAR_Y, C_THRES);
    show_image("after", image, true);

    //**********************************************************
    //******************* Algorithms ***************************
    //**********************************************************

    //For storage of final points used to fit the polynomal
    std::vector<Point2f> left_points;
    std::vector<Point2f> right_points;

    //Return code
    int code;

    //Partitioned Hough
    if (ALGO == 1)
    {
        code = hough(image, left_points, right_points, NUM_PART, B_VIEW, ROI_START);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    //ALM
    if (ALGO == 2)
    {
        code = alm(image, left_points, right_points, NUM_PART, NUM_LINES, B_VIEW, ROI_START);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    //Sliding Windows
    if (ALGO == 3)
    {
        code = sliding_windows_search(image, ROI_START, W_NUM_WINDOWS, W_WIDTH, left_points, right_points);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    //Window search
    if (ALGO == 4)
    {
        code = window_search(image, W_WIDTH, ROI_START, left_points, right_points);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    //**********************************************************
    //******************* Fitting and Storing ******************
    //**********************************************************

    std::vector<double> right_coeff;
    std::vector<double> left_coeff;

    poly_reg(left_points, right_points, left_coeff, right_coeff, ORDER);

    if (B_VIEW)
    {
#ifndef NDEBUG
        draw_poly(image, 0, left_coeff, right_coeff, ORDER);
        show_image("bird", image, true);
        draw_poly(clone, ROI_START, left_coeff, right_coeff, ORDER, b_inv_mat);
        show_image("normal", clone, true);
#endif
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, input_file, b_inv_mat);
        if (code != MAPRA_SUCCESS)
            return code;
    }
    //in normal-view
    else
    {
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, input_file);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    return MAPRA_SUCCESS;
}