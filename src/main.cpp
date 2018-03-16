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
    if (argc != 5)
    {
        std::cout << "not right amount of args" << std::endl;
        std::cout << "Call like this: ./mapra <input_directory_path> <input_file_name> <result_directory_path> <parameterFile_path>" << std::endl;
        return MAPRA_ERROR;
    }
    String input_dir = modify_dir(argv[1]);
    String input_file = argv[2];
    String result_dir = modify_dir(argv[3]);
    String parameter_file = argv[4];
    //check if an actual parameterfile is given
    if(parameter_file.find(".par") == String::npos)
    {
        std::cout << "wrong parameter file given" << std::endl;
        return MAPRA_ERROR;
    }
#ifndef NDEBUG
    std::cout << "input_dir: " << input_dir << ", result_dir: " << result_dir << ", paramter_file: " << parameter_file << std::endl;
#endif
    String image_location = input_dir + input_file;
    //finds number part of input file name
    String png_number = input_file.substr(input_file.find("_")+1, input_file.find(".png")-input_file.find("_")-1);
    //finds prefix part of input file name; used later for the right output file name
    String png_prefix = input_file.substr(0, input_file.find("_"));
    //evaluation category is always "road" and not "lane"
    String output_file = png_prefix + "_road_" + png_number + ".png";
    Mat image;
    Mat clone;
    Mat calibration; //used for b_view_calibration()
    image = imread(image_location, 1);
#ifndef NDEBUG
    clone = image.clone();
#endif


    if (!image.data)
    {
        //printf("No image data\n");
        std::cout << "no image data" << std::endl;
        return MAPRA_ERROR;
    }

    //#############################################################################################
    //######################################### Parameter File ####################################
    //#############################################################################################

    //create a ParameterReader
    Parameter_reader parameter;
    parameter.read(parameter_file);

    
    //1 = part. Hough, 2 = ALM, 3 = Sliding Window, 4 = Multiple Window
    const int ALGO = parameter.get_value<int>("algo");     //1,2,3,4
    //Amount of partitions and lines
    const int NUM_PART = parameter.get_value<int>("num_part"); //2-5, for algo 1,2
    const int NUM_LINES = parameter.get_value<int>("num_lines"); //2-5, for algo 2
    //Sliding Window Constants
    const int W_NUM_WINDOWS = parameter.get_value<int>("w_num_windows"); //3,5,7,9,11 for algo 3
    const int W_WIDTH = parameter.get_value<int>("w_width");       //20, 40, 60, 80, for algo 3,4

    //Birdview Constants
    //use b_view_calibration() once to get initial values for the 4 parameters for the used camera setup
    const bool B_VIEW = parameter.get_value<bool>("b_view"); //true, false
    const double B_OFFSET_MID = 0.04;  //const
    const double B_OFFSET = 0.21;      //const
    const double B_OFFSET2 = 0.04;     //const
    const double B_HEIGHT = 0.52;      //const
    Mat b_mat;
    Mat b_inv_mat;

    //manually set once according to used camera setup
    const double ROI_START = B_VIEW ? 0. : 0.52;

    //Edge detection filters
    //1 = canny, 2 = sobel_mag, 3 = sobel_par, 4 = color_thres
    //1; 2; 3; 4; 1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 1,2,3; 2,3,4; 1,3,4; 1,2,4; 1,2,3,4
    std::vector<int> FILTERS;
    if (parameter.get_value<int>("filter1") == 1)
        FILTERS.push_back(1);
    if (parameter.get_value<int>("filter2") == 1)
        FILTERS.push_back(2);
    if (parameter.get_value<int>("filter3") == 1)
        FILTERS.push_back(3);
    if (parameter.get_value<int>("filter4") == 1)
        FILTERS.push_back(4);
    assert(FILTERS.size() >= 1);
    if(FILTERS.size() == 0u)
    {
        std::cout << "error in filter read in" << std::endl;
        return MAPRA_ERROR;
    }

    //Canny Parameter
    int CA_THRES;   //const
    int KERNEL;     //const
    //Sobel Thresholding Parameter
    int S_MAG;      //const
    int S_PAR_X;    //const
    int S_PAR_Y;    //const 
    //Color Thresholding Parameter
    int C_THRES;    //const

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
        CA_THRES = 100;
        KERNEL = 3;
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
    int order = parameter.get_value<int>("order"); //2,3
    const int ORDER = (order > NUM_PART && NUM_PART != -1) ? NUM_PART : order;
    std::cout << "order: " << ORDER << std::endl;

    //#############################################################################################
    //######################################### Program ###########################################
    //#############################################################################################

    //**********************************************************
    //****************** Preliminary Setup *********************
    //**********************************************************

    //Use once for calibration of the b_view parameters
    //Comment out for actual lane detection
    //calibration = image.clone(); //used for b_view_calibration()
    //b_view_calibration(&calibration, B_OFFSET_MID, B_OFFSET, B_OFFSET2, B_HEIGHT);

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
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, output_file, b_inv_mat);
        if (code != MAPRA_SUCCESS)
            return code;
    }
    //in normal-view
    else
    {
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, output_file);
        if (code != MAPRA_SUCCESS)
            return code;
    }

    return MAPRA_SUCCESS;
}