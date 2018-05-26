#include "filters.hpp"

//#############################################################################################
//######################################### STATIC FCTS #######################################
//#############################################################################################

//#############################################################################################
//######################################### INTERFACE #########################################
//#############################################################################################

void bird_view(const Mat &input_img, Mat &output_img, const double rel_height, const double rel_left, const double rel_right)
{
    double offset = 0.4;
    double offset2 = 0.05;
    //construct two trapezoids
    Point2f p1[4] = {Point2f(rel_left * input_img.cols, rel_height * input_img.rows), Point2f(rel_right * input_img.cols, rel_height * input_img.rows), Point2f((rel_right + offset) * input_img.cols, input_img.rows), Point2f((rel_left - offset) * input_img.cols, input_img.rows)};
    Point2f p2[4] = {Point2f((rel_left - offset2) * input_img.cols, 0), Point2f((rel_right + offset2) * input_img.cols, 0), Point2f((rel_right + offset2) * input_img.cols, input_img.rows), Point2f((rel_left - offset2) * input_img.cols, input_img.rows)};

    Mat mat = getPerspectiveTransform(p1, p2);
    warpPerspective(input_img, output_img, mat, Size(input_img.cols, input_img.rows));
}

void canny_blur(Mat &image, const int thres, const int kernel)
{
    //tmp image
    Mat proc;
    //convert from color to gray (one channel, 8bit)
    cvtColor(image, proc, COLOR_BGR2GRAY);
    //blur image
    blur(proc, proc, Size(3, 3));
    //apply canny edge detection
    Canny(proc, image, thres, thres * 3, kernel);
}

void color_thres(Mat &image, const int thres)
{
    //convert to HLS color space
    cvtColor(image, image, COLOR_BGR2HLS);
    //binary (one channel) temporary Mat
    Mat tmp(image.rows, image.cols, CV_8UC1);
    //take second channel (1) (L channel) and store in in first channel (0) in new image
    int from_to[] = {1, 0};
    //extract second channel from image and save to first channel of tmp
    mixChannels(&image, 1, &tmp, 1, from_to, 1);
    //reshape from 3 channels to 1 channel in order to hold the newly created tmp
    image.reshape(1);
    image = tmp;

    double max_val;
    //get maximum value from image
    minMaxLoc(image, (double *)0, &max_val);
    //normalize image
    image *= (255. / max_val);
    //apply binary thresholding
    threshold(image, image, thres, 255, THRESH_BINARY);
}

//deprecated
void gabor(Mat &image)
{
    int kernel_size = 40;
    double sigma = 1;         //frequency bandwidth
    double theta = 0.;        //angle of gabor kernel --> 0 for vertical lines
    double lambda = 20.;      //double the lane width in pixles
    double gamma = 0.5;       //shape (1 = circular or [0,1) = elliptical)
    double psi = 0.5 * CV_PI; //phase offset
    Mat kernel1 = getGaborKernel(Size(kernel_size, kernel_size), sigma, theta, lambda, gamma, psi, CV_32F);
    Mat kernel2 = getGaborKernel(Size(kernel_size, kernel_size), sigma, theta, lambda, gamma, 0, CV_32F);
#ifndef NDEBUG
    Mat kernel3;
    resize(kernel1, kernel3, Size(500, 500), 0, 0);
    Mat img;
    normalize(kernel3, img, 0, 1, NORM_MINMAX);
    show_image("Gabor Kernel enlarged", img, true);
#endif
    filter2D(image, image, CV_32F, kernel2);
    filter2D(image, image, CV_32F, kernel1);
}

void h_sobel(Mat &image)
{
    //consider only horizontal edges
    Sobel(image, image, -1, 1, 0);
}

void multi_filter(Mat &image, std::vector<int> algos, int ca_thres, int kernel, int s_mag, int r_thres, int r_tau, int c_thres)
{
    assert(*std::max_element(algos.begin(), algos.end()) <= 4 && *std::min_element(algos.begin(), algos.end()) >= 1);
    using namespace std::placeholders;

    //add pair of wanted function and image Mat to call it on to a vector
    //bind a copy of image to the function argument with a placeholder, rest of necessary arguments with correct other arguments
    std::vector<std::pair<std::function<void(Mat &)>, Mat>> fct_calls;
    if (any_of(algos.begin(), algos.end(), [](int i) { return i == 1; }))
        fct_calls.push_back(std::make_pair(std::bind(canny_blur, _1, ca_thres, kernel), image.clone()));
    if (any_of(algos.begin(), algos.end(), [](int i) { return i == 2; }))
        fct_calls.push_back(std::make_pair(std::bind(sobel_mag_thres, _1, s_mag), image.clone()));
    if (any_of(algos.begin(), algos.end(), [](int i) { return i == 3; }))
        fct_calls.push_back(std::make_pair(std::bind(row_filter, _1, r_thres, r_tau), image.clone()));
    if (any_of(algos.begin(), algos.end(), [](int i) { return i == 4; }))
        fct_calls.push_back(std::make_pair(std::bind(color_thres, _1, c_thres), image.clone()));

    //call functions in vector one after another
    //processed image stays in second entry of pair
    for (auto &f : fct_calls)
        f.first(f.second);

    //convert original image to gray (single channel, 8bit)
    cvtColor(image, image, COLOR_BGR2GRAY);
    //set all pixels to white (=true =255)
    image = Scalar(255);
    for (auto &f : fct_calls)
    {
#ifndef NDEBUG
        static int i = 0;
        //show_image("multi_filter: " + std::to_string(i), image, true);
        show_image(std::to_string(i), f.second, true);
        ++i;
#endif
        //get second entry of pair (processed copied image) and 
        //unify individual pixels with original image pixels
        bitwise_and(image, f.second, image);
    }
}



//deprecated
void sobel_dir_thres(Mat &image, const int thres_1, const int thres_2)
{
    //helper needed for temporary conversion to CV_32F from CV_8U
    Mat tmp;
    Mat tmp2;
    Mat tmp3;
    cvtColor(image, image, COLOR_BGR2GRAY);
    blur(image, image, Size(3, 3));
    Mat image2 = image.clone();
    //x
    Sobel(image, tmp, CV_32F, 1, 0, 5);
    //y
    Sobel(image2, tmp2, CV_32F, 0, 1, 5);
    //angle between x and y stored in tmp
    phase(tmp, tmp2, tmp3, true);
    //scales tmp3 from [0,360] to image with [0,255] and converts to 8 bit
    convertScaleAbs(tmp3, image);
    //remove (set to 0) all values over thres_e, leave other ones untouched
    //scales the degrees of thres_e to [0,255]
    threshold(image, image, (thres_1 + 15.) / 360. * 255., 0, THRESH_TOZERO_INV);
    //remove (set to 0) all values under thres_s, leave other ones untouched
    threshold(image, image, (thres_1 - 15.) / 360. * 255., 0, THRESH_TOZERO);
    //threshold is 0 --> all values over 0 are set to 255
    threshold(image, image, 0, 255, THRESH_BINARY);
    //analogly for second direction thres_2
    convertScaleAbs(tmp3, image2);
    threshold(image2, image2, (thres_2 + 15.) / 360. * 255., 0, THRESH_TOZERO_INV);
    threshold(image2, image2, (thres_2 - 15.) / 360. * 255., 0, THRESH_TOZERO);
    threshold(image2, image2, 0, 255, THRESH_BINARY);

    bitwise_or(image, image2, image);
}

void sobel_mag_thres(Mat &image, const int thres)
{
    //helper needed for temporary conversion to CV_32F from CV_8U
    Mat tmp;
    Mat tmp2;
    cvtColor(image, image, COLOR_BGR2GRAY);
    blur(image, image, Size(3, 3));
    Mat image2 = image.clone();
    //Sobel derivatives in x direction
    Sobel(image, image, -1, 1, 0);
    Sobel(image2, image2, -1, 0, 1);
    image.convertTo(tmp, CV_32F);
    image2.convertTo(tmp2, CV_32F);
    //get magnitude of x and y derivaties
    magnitude(tmp, tmp2, tmp);
    tmp.convertTo(image, CV_8U);
    double max_val;
    //find max value in image
    minMaxLoc(image, (double *)0, &max_val);
    //normalize image
    image *= (255. / max_val);
    threshold(image, image, thres, 255, THRESH_BINARY);
}

//deprecated
void sobel_par_thres(Mat &image, const int thres_x, const int thres_y)
{
    cvtColor(image, image, COLOR_BGR2GRAY);
    blur(image, image, Size(3, 3));
    Mat image2 = image.clone();
    //Sobel derivatives in x direction
    Sobel(image, image, -1, 1, 0);
    absdiff(image, Scalar::all(0), image);
    double max_val;
    //get maximum value from image
    minMaxLoc(image, (double *)0, &max_val);
    //normalize image
    image *= (255. / max_val);
    threshold(image, image, thres_x, 255, THRESH_BINARY);
    //same for image2 in y direction
    Sobel(image2, image2, -1, 0, 1);
    absdiff(image2, Scalar::all(0), image2);
    //get maximum pixel value of image2
    minMaxLoc(image2, (double *)0, &max_val);
    image2 *= (255. / max_val);
    threshold(image2, image2, thres_y, 255, THRESH_BINARY);
    bitwise_and(image, image2, image);
}
