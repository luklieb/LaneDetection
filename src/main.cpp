#include <iostream>
#include <functional>
#include <chrono>
#include <fstream>
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

class Timer
{
  private:
    typedef std::chrono::high_resolution_clock Clock_T;

  public:
    Timer() : start_(Clock_T::now()) {}

    void reset() { start_ = Clock_T::now(); }

    /// Returns the elapsed time in seconds since either the construction of the object or the last call to reset()
    double elapsed() const
    {
        using namespace std::chrono;
        return duration_cast<duration<double, seconds::period>>(Clock_T::now() - start_).count();
    }

  private:
    Clock_T::time_point start_;
};

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        std::cerr << "not right amount of args" << std::endl;
        std::cerr << "Call like this: ./lanedet <input_image_directory_path> <input_image_file_name> <result_directory_path> <parameter_file_path> <time_ouput_directory_path>" << std::endl;
        std::cerr << "For Example:    ./lanedet ../eval/eval_images/input/ um_000001.png ../eval/eval_images/tmp ../eval/eval_param/param_123.par ../eval/eval_results/" << std::endl;
        return LANEDET_ERROR;
    }
    String input_dir = modify_dir(argv[1]);
    String input_file = argv[2];
    String result_dir = modify_dir(argv[3]);
    String parameter_file = argv[4];
    String time_dir = modify_dir(argv[5]);
    //check if an actual parameterfile is given
    if (parameter_file.find(".par") == String::npos)
    {
        std::cerr << "wrong parameter file given" << std::endl;
        return LANEDET_ERROR;
    }
    String param_number = parameter_file.substr(parameter_file.find("param_") + 6, parameter_file.find(".par") - parameter_file.find("param_") - 6);
#ifndef NDEBUG
    std::cout << "input_dir: " << input_dir << ", result_dir: " << result_dir << ", paramter_file: " << parameter_file << std::endl;
#endif
    String image_location = input_dir + input_file;
    //finds number part of input file name
    String png_number = input_file.substr(input_file.find("_") + 1, input_file.find(".png") - input_file.find("_") - 1);
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
        std::cerr << "no image data" << std::endl;
        return LANEDET_ERROR;
    }

    //#############################################################################################
    //######################################### Parameter File ####################################
    //#############################################################################################

    //create a ParameterReader
    Parameter_reader parameter;
    parameter.read(parameter_file);

    //1 = part. Hough, 2 = ALM, 3 = Sliding Window, 4 = Multiple Window, 5 = Random lines
    const int ALGO = parameter.get_value<int>("algo"); //1,2,3,4
    //Amount of partitions and lines
    const int NUM_PART = parameter.get_value<int>("num_part"); //2-5, for algo 1,2,5
    const std::vector<int> allowed_num_part = {2, 3, 4, 5};
    const int NUM_LINES = parameter.get_value<int>("num_lines"); //2-5, for algo 2
    const std::vector<int> allowed_num_lines = {2, 3, 4, 5};
    //Sliding Window Constants
    const int W_NUM_WINDOWS = parameter.get_value<int>("w_num_windows"); //4,5,7,9,11 for algo 3
    const std::vector<int> allowed_w_num_windows = {4, 5, 7, 9, 11};
    const int W_WIDTH = parameter.get_value<int>("w_width"); //20, 40, 60, 80, for algo 3,4
    const std::vector<int> allowed_w_width = {20, 40, 60, 80};

    //Random search constants
    const int R_NUM_LINES = parameter.get_value<int>("r_num_lines"); //200, 400, 600, 800 for algo 5
    const std::vector<int> allowed_r_num_lines = {200, 400, 600, 800, 8000};

    //Birdview Constants manually set once
    //use b_view_calibration() once to get initial values for the 4 parameters for the used camera setup
    const bool B_VIEW = parameter.get_value<bool>("b_view"); //true, false
    const std::vector<bool> allowed_b_view = {true, false};
    const double B_OFFSET_MID = 0.04; //const
    const double B_OFFSET = 0.21;     //const
    const double B_OFFSET2 = 0.04;    //const
    const double B_HEIGHT = 0.52;     //const
    Mat b_mat;
    Mat b_inv_mat;

    //manually set once according to used camera setup
    const double ROI_START = B_VIEW ? 0. : 0.52;

    //Edge detection filters
    //1 = canny, 2 = sobel_mag, 3 = row_filter, 4 = color_thres
    //1; 2; 3; 4; 1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 1,2,3; 2,3,4; 1,3,4; 1,2,4; 1,2,3,4
    const std::vector<std::vector<int>> allowed_filters = {{1}, {2}, {3}, {4}, {1, 2}, {1, 3}, {1, 4}, {2, 3}, {2, 4}, {3, 4}, {1, 2, 3}, {2, 3, 4}, {1, 3, 4}, {1, 2, 4}, {1, 2, 3, 4}};
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
    if (FILTERS.size() == 0u)
    {
        std::cerr << "error in filter read in" << std::endl;
        return LANEDET_ERROR;
    }

    //Canny Parameter
    int CA_THRES; //const
    int KERNEL;   //const
    //Sobel Thresholding Parameter
    int S_MAG; //const
    //Row Filter Parameter
    int R_THRES;
    int R_TAU;
    //Color Thresholding Parameter
    int C_THRES; //const

    //parameters tuned by hand according to 4 different scenarios,
    //which have the largest effect on filters
    if (!B_VIEW && FILTERS.size() > 1)
    {
        CA_THRES = 40;
        KERNEL = 3;
        S_MAG = 90;
        R_THRES = 50;
        R_TAU = 20;
        C_THRES = 150;
    }
    if (B_VIEW && FILTERS.size() > 1)
    {
        CA_THRES = 250;
        KERNEL = 5;
        S_MAG = 50;
        R_THRES = 10;
        R_TAU = 10;
        C_THRES = 150;
    }
    if (!B_VIEW && FILTERS.size() == 1)
    {
        CA_THRES = 100;
        KERNEL = 3;
        S_MAG = 240;
        R_THRES = 75;
        R_TAU = 20;
        C_THRES = 225;
    }
    if (B_VIEW && FILTERS.size() == 1)
    {
        CA_THRES = 350;
        KERNEL = 5;
        S_MAG = 125;
        R_THRES = 50;
        R_TAU = 10;
        C_THRES = 210;
    }

    //Fitting Constants
    int order = parameter.get_value<int>("order"); //2,3
    const int ORDER = (order > NUM_PART && NUM_PART != -1) ? NUM_PART : order;
    const std::vector<int> allowed_order = {2, 3};

    //check if commenly used parameters are set correctly
    //other parameters are tested further below at the respective algo
    if (
        !check_param(B_VIEW, allowed_b_view) ||
        !check_param(FILTERS, allowed_filters) ||
        !check_param(ORDER, allowed_order))
    {
        std::cerr << "wrong commen parameters in param_file: " << parameter_file << std::endl;
        return LANEDET_ERROR;
    }

    //#############################################################################################
    //######################################### Program ###########################################
    //#############################################################################################

    //**********************************************************
    //****************** Preliminary Setup *********************
    //**********************************************************

    //Use once for calibration of the b_view parameters
    //Comment next two lines out for actual lane detection
    //calibration = image.clone(); //used for b_view_calibration()
    //b_view_calibration(&calibration, B_OFFSET_MID, B_OFFSET, B_OFFSET2, B_HEIGHT);


    cvtColor(image.clone(), clone, COLOR_BGR2GRAY);

    Timer time_measurement;

    if (B_VIEW)
    {
        //both trapezoids have to be adjusted according to the size of the input image and the ROI
        //In the end, both road lanes should be parallel in the Birds Eye View
        //Use b_view_calibration() once to get these constants
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
#ifndef NDEBUG
        show_image("after bird view", image, true);
#endif
    }

    //**********************************************************
    //****************** Edge Detection ************************
    //**********************************************************

    multi_filter(image, FILTERS, CA_THRES, KERNEL, S_MAG, R_THRES, R_TAU, C_THRES);
#ifndef NDEBUG
    show_image("after multi filter", image, true);
#endif

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
        if (!check_param(NUM_PART, allowed_num_part))
        {
            std::cerr << "wrong parameters in algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            return LANEDET_ERROR;
        }

        code = hough(image, left_points, right_points, NUM_PART, B_VIEW, ROI_START);

        if (code != LANEDET_SUCCESS)
        {
            std::cout << "something went wrong in Algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return code;
        }
    }
    //ALM
    else if (ALGO == 2)
    {
        if (
            !check_param(NUM_PART, allowed_num_part) ||
            !check_param(NUM_LINES, allowed_num_lines))
        {
            std::cerr << "wrong parameters in algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return LANEDET_ERROR;
        }

        code = alm(image, left_points, right_points, NUM_PART, NUM_LINES, B_VIEW, ROI_START);

        if (code != LANEDET_SUCCESS)
        {
            std::cout << "something went wrong in Algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return code;
        }
    }
    //Sliding Windows
    else if (ALGO == 3)
    {
        if (
            !check_param(W_NUM_WINDOWS, allowed_w_num_windows) ||
            !check_param(W_WIDTH, allowed_w_width))
        {
            std::cerr << "wrong parameters in algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            return LANEDET_ERROR;
        }

        code = sliding_windows_search(image, ROI_START, W_NUM_WINDOWS, W_WIDTH, left_points, right_points);

        if (code != LANEDET_SUCCESS)
        {
            std::cout << "something went wrong in Algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return code;
        }
    }
    //Window search
    else if (ALGO == 4)
    {
        if (!check_param(W_WIDTH, allowed_w_width))
        {
            std::cerr << "wrong parameters in algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            return LANEDET_ERROR;
        }

        code = fixed_window_search(image, W_WIDTH, ROI_START, left_points, right_points);

        if (code != LANEDET_SUCCESS)
        {
            std::cout << "something went wrong in Algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return code;
        }
    }
    //Random search
    else if (ALGO == 5)
    {
        if (
            !check_param(R_NUM_LINES, allowed_r_num_lines) ||
            !check_param(NUM_PART, allowed_num_part))
        {
            std::cerr << "wrong parameters in algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            return LANEDET_ERROR;
        }

        code = random_search(image, R_NUM_LINES, ROI_START, NUM_PART, B_VIEW, left_points, right_points);

        if (code != LANEDET_SUCCESS)
        {
            std::cout << "something went wrong in Algo " << ALGO << ", and param_file: " << parameter_file << std::endl;
            store_void_result(image, result_dir, output_file);
            return code;
        }
    }
    else
    {
        std::cerr << "wrong Algo given" << std::endl;
        return LANEDET_ERROR;
    }

    //**********************************************************
    //******************* Fitting and Storing ******************
    //**********************************************************

    std::vector<double> right_coeff;
    std::vector<double> left_coeff;

    poly_reg(left_points, right_points, left_coeff, right_coeff, ORDER);

    //stop time measurement
    //append measured time to time file
    double time_final = time_measurement.elapsed();
    std::ofstream time_file;
    time_file.open(time_dir + "time" + param_number, std::ios::app);
    time_file << time_final << std::endl;
    time_file.close();
    std::cout << "time in sec: " << time_final << std::endl;

    if (B_VIEW)
    {
#ifndef NDEBUG
        draw_poly(image, 0, left_coeff, right_coeff, ORDER);
        show_image("bird", image, true);
        draw_poly(clone, ROI_START, left_coeff, right_coeff, ORDER, b_inv_mat);
        show_image("normal", clone, true);
#endif
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, output_file, b_inv_mat);
        if (code != LANEDET_SUCCESS)
            return code;
    }
    //in normal-view
    else
    {
        code = store_result(image, ROI_START, left_coeff, right_coeff, ORDER, result_dir, output_file);
        if (code != LANEDET_SUCCESS)
            return code;
    }
    std::cout << clone.channels() << std::endl;
    return LANEDET_SUCCESS;
}
