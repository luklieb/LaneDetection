#include <iostream>
#include <functional>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "algos.hpp"
#include "helper.hpp"

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
    std::cout << input_dir << ", " << result_dir << std::endl;
    String image_location = input_dir + input_file;
    Mat image, processed, bird;
    image = imread(image_location, 1);

    if (!image.data)
    {
        printf("No image data\n");
        return MAPRA_ERROR;
    }


    //#############################################################################################
    //######################################### Parameter File ####################################
    //#############################################################################################
    
    //1 = part. Hough, 2 = ALM, 3 = Sliding Window, 4 = Multiple Window
    const int ALGO = 1; //1,2,3,4
    const double ROI_START = 0.6; //const
    //Amount of partitions and lines
    const int NUM_PART = 2; //1-5
    const int NUM_LINES = 3; //2-5
    //Sliding Window Constants
    const int W_NUM_WINDOWS = 10; //3-10
    const int W_WIDTH = 40;       //20, 40, 60, 80

    //Birdview Constants
    const bool B_VIEW = true; //true, false
    const double B_OFFSET_MID = 0.04; //const
    const double B_OFFSET = 0.4; //const
    const double B_OFFSET2 = 0.05; //const
    const double B_HEIGHT = ROI_START; //const
    Mat b_mat;
    Mat b_inv_mat;

    int P_START;
    if (B_VIEW)
    {
        P_START = 0;
    }
    else
    {
        P_START = ROI_START * image.rows;
    }

    //Edge detection filters
    //1 = canny, 2 = sobel_mag, 3 = sobel_par, 4 = color_thres
    //1; 2; 3; 4; 1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 1,2,3; 2,3,4; 1,3,4; 1,2,4; 1,2,3,4
    const std::vector<int> FILTERS = {1, 2}; 
    assert(FILTERS.size() >= 1);
    //tmp variables for filters; used to finetune parameters for
    //one filter or when more than one filter is used
    int ca_th = 100;
    int k = 3;
    int s_m = 155;
    int s_p_x = 10;
    int s_p_y = 60;
    int c_th = 220;
    if(FILTERS.size() > 1)
    {
        ca_th = 100;
        k = 3;
        s_m = 155;
        s_p_x = 10;
        s_p_y = 60;
        c_th = 220;
    }
    //Canny Constants
    const int CA_THRES = ca_th;
    const int KERNEL = k; //3, 5, 7

    //Sobel Thresholding Constants
    const int S_MAG = s_m;
    const int S_PAR_X = s_p_x;
    const int S_PAR_Y = s_p_y;

    //Color Thresholding Constants
    const int C_THRES = c_th;


    //Fitting Constants
    const int ORDER = NUM_PART; //const

    //#############################################################################################
    //######################################### Program ###########################################
    //#############################################################################################


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

    //Partitioned Hough
    if (ALGO == 1)
    {
        int coords_part[NUM_PART + 1];
        sub_partition(P_START, image.rows, NUM_PART, true, coords_part);
        std::vector<Vec2f> left_lines;
        std::vector<Vec2f> right_lines;
        partitioned_hough(processed, coords_part, NUM_PART, 1, left_lines, right_lines, B_VIEW);
        get_points(left_lines, right_lines, NUM_LINES, NUM_PART, coords_part, left_points, right_points);
        //maybe add an average function for the points on same line...
    }

    //ALM
    if (ALGO == 2)
    {
        int coords_part[NUM_PART + 1];
        sub_partition(P_START, image.rows, NUM_PART, true, coords_part);
        std::vector<Vec2f> left_lines;
        std::vector<Vec2f> right_lines;
        partitioned_hough(processed, coords_part, NUM_PART, NUM_LINES, left_lines, right_lines, B_VIEW);
        get_points(left_lines, right_lines, NUM_LINES, NUM_PART, coords_part, left_points, right_points);
        alm(left_points, right_points, NUM_PART, NUM_LINES);
        alm_conversion(left_points, right_points);
    }

    //Sliding Windows
    if(ALGO == 3)
    {
        sliding_windows_search(processed, ROI_START, W_NUM_WINDOWS, W_WIDTH, left_points, right_points);
    }

    //Window search
    if(ALGO == 4)
    {
        int histo_points [2];
        h_histogram(processed, ROI_START, histo_points);
        window_search(processed, histo_points, W_WIDTH, ROI_START, left_points, right_points);
    }

    //**********************************************************
    //******************* Fitting and Storing ******************
    //**********************************************************

    std::vector<double> right_coeff;
    std::vector<double> left_coeff;
   
    if (B_VIEW)
    {
        //transform points from Bird View to regular view
        std::vector<Point2f> tmp1 (left_points);
        std::vector<Point2f> tmp2 (right_points);
        perspectiveTransform(tmp1, left_points, b_inv_mat);
        perspectiveTransform(tmp2, right_points, b_inv_mat);
    }
    poly_reg(left_points, right_points, left_coeff, right_coeff, ORDER);
    store_result(processed, ROI_START, left_coeff, right_coeff, ORDER, result_dir, input_file);

    return MAPRA_SUCCESS;
}