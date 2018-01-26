#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "algos.hpp"

using namespace cv;

/**
 * global coord system
 * 		  x 
 *	 -|--------> 
 * y  |
 * 	  |
 * 	  v
 */

/**
 * Helper function to show an image
 * @param image_name for the window
 * @param image to be shwon in window
 * @param wait option to wait for a key input to close the window showing the image
 */
void show_image(String image_name, Mat &image, bool wait)
{
	namedWindow(image_name, WINDOW_AUTOSIZE);
	imshow(image_name, image);
	if (wait)
		waitKey(0);
}

/**
 * Creates a Gabor kernel and applies it to the input image for edge detection
 * @param image being converted to an edge image
 * @note compare to canny_blur(Mat&)
 */
void gabor(Mat &image)
{
	//TODO passende parameter finden
	int kernel_size = 50;
	double sigma = 5;		  //frequency bandwidth
	double theta = 0.;		  //angle of gabor kernel --> 0 for vertical lines
	double lambda = 20.;	  //double the lane width in pixles
	double gamma = 0.;		  //shape (circular or elliptical)
	double psi = 0.5 * CV_PI; //phase offset
	Mat kernel = getGaborKernel(Size(kernel_size, kernel_size), sigma, theta, lambda, gamma, psi, CV_32F);
	//resize(kernel, kernel, Size(500,500), 0, 0);
	//Mat img;
	//normalize(kernel, img, 0, 1, NORM_MINMAX);
	//show_image("bla", img, true);
	filter2D(image, image, CV_64F, kernel);
}

/**
 * Blurs the input image and applies Canny edge detection on it 
 * @param image being converted to an edge image
 * @note compare to gabor(Mat&)
 */
void canny_blur(Mat &image)
{
	//original: low_threshold=50, kernel_size=3: smooth, but many left out edges
	int low_threshold = 350;
	int kernel_size = 5;
	blur(image, image, Size(3, 3));
	Canny(image, image, low_threshold, low_threshold * 3, kernel_size);
}

/**
 * Returns the (x or y) coordinates of the sub-domains
 * @param start of the whole domain to be partitioned
 * @param end of the whole domain to be partitioned
 * @param euqidistant whether or not sub-domain borders should be euqidistant
 * @param number of sub-domains
 * @param coords array of length number+1, returns the coordinates of the sub-domain borders (including start and end)
 */
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

/**
 * Removes only (!) horizontal lines in the input image 
 * @param image in which horizontal lines should be removed
 */
void h_sobel(Mat &image){
	Sobel(image, image, -1, 1, 0);
}

/**
 * Applies the Hough Transformation on different sub-regions 
 * @param img The image for the Hough Transformation
 * @param part_coords Holds the num_part+1 coordinates of the partitions
 * @param num_part Number of sub-domains/partitions
 * @param num_lines Number of lines per partition per side 
 * @param left_lines Vector holding the polar coordinates of the detected lines on the left side (lane)
 * @param right_lines Vector holding the polar coordinates of the detected lines on the right side (lane)
 * @note [left/right]_lines stores at the beginning num_lines lines of the first partition,
 * 		at the end num_lines lines of the last partition
 */
void partitioned_hough(const Mat &img, const int *part_coords, const int num_part, const int num_lines, std::vector<Vec2f> &left_lines, std::vector<Vec2f> &right_lines)
{
	left_lines.clear();
	right_lines.clear();
	std::vector<Vec2f> left_lines_tmp;
	std::vector<Vec2f> right_lines_tmp;
	for(int i = 0; i<num_part; ++i)
	{
		HoughLinesCustom(img, 1., CV_PI / 180., 10, left_lines_tmp, right_lines_tmp, num_lines, part_coords[i], part_coords[i+1]);
		left_lines.insert(left_lines.end(), left_lines_tmp.begin(), left_lines_tmp.end());
		right_lines.insert(right_lines.end(), right_lines_tmp.begin(), right_lines_tmp.end());
		left_lines_tmp.clear();
		right_lines_tmp.clear();
	}
}

/**
 * Converts the polar coordiantes from a Hough Transform (lines) to points according to their start/end of each partition
 * @param lines The polar coordinates returned from a Hough Transform
 * @param num_lines The number of lines of each side of each partition
 * @param coords_part Coordinates of the partitions
 * @param points Holds the converted Points 
 */
void get_points(const std::vector<Vec2f> &left_lines, const std::vector<Vec2f> &right_lines, const int num_lines, const int *coords_part, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{

	left_points.clear();
	right_points.clear();
	int i = 0;
	int j = 0;
	for (auto p : left_lines)
	{
		//std::cout << p[0] << ", " << p[1] / CV_PI << std::endl;
		float rho = p[0], theta = p[1];
		Point2f pt1, pt2;
		double a = cos(theta), b = sin(theta), a_inv = 1./a;;
		pt1.y = coords_part[i];
		pt1.x = (rho - pt1.y*b)*a_inv;
		pt2.y = coords_part[i+1];	
		pt2.x = (rho - pt2.y*b)*a_inv;
		left_points.push_back(pt1);
		left_points.push_back(pt2);
		++j;
		if(j%num_lines == 0)
			++i;
	}

	i = 0;
	j = 0;
	for (auto p : right_lines)
	{
		//std::cout << p[0] << ", " << p[1] / CV_PI << std::endl;
		float rho = p[0], theta = p[1];
		Point2f pt1, pt2;
		double a = cos(theta), b = sin(theta), a_inv = 1./a;;
		pt1.y = coords_part[i];
		pt1.x = (rho - pt1.y*b)*a_inv;
		pt2.y = coords_part[i+1];	
		pt2.x = (rho - pt2.y*b)*a_inv;
		right_points.push_back(pt1);
		right_points.push_back(pt2);
		++j;
		if(j%num_lines == 0)
			++i;
	}
}


int main(int argc, char **argv)
{
	char *image_name = argv[1];
	Mat image, processed, bird;
	image = imread(image_name, 1);
	Mat mask = Mat(Size(image.cols, image.rows), image.type());
	if (argc != 2 || !image.data)
	{
		printf(" No image data \n ");
		return -1;
	}

	/**
	 * random
	 */
	//int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	//double fontScale = 2;
	//int thickness = 3;
	//putText(processed, std::to_string(number), p, fontFace, fontScale, Scalar::all(255), thickness, 8);

	/**
	 * To be read in from parameter file
	 */
	const double ROI_START = 0.6;
	const int NUM_PART = 5;
	const int NUM_LINES = 3;
	const double B_OFFSET_MID = 0.04;
	const double B_OFFSET = 0.4;
    const double B_offset2 = 0.05;
    const double B_HEIGHT = ROI_START; //or: ROI_START
	const int W_NUM_WINDOWS = 10;
	const int W_WIDTH = 40;

	/**
	 * Program
	 */

	const Point2f b_p1[4] = {Point2f((0.5-B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID + B_OFFSET)*image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_OFFSET)*image.cols, image.rows)};
    const Point2f b_p2[4] = {Point2f((0.5-B_OFFSET_MID-B_offset2)*image.cols, 0), Point2f((0.5+B_OFFSET_MID+B_offset2)*image.cols, 0), Point2f((0.5+B_OFFSET_MID+B_offset2)*image.cols, image.rows), Point2f((0.5-B_OFFSET_MID-B_offset2)*image.cols, image.rows)};
	const Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
	const Mat b_inv_mat = getPerspectiveTransform(b_p2, b_p1);
	warpPerspective(image, bird, b_mat, Size(image.cols, image.rows));
	show_image("bird", bird, true);	
	

	/* //IMPORTANT!!!
	std::vector<Point2f> points_in_bird_view = {Point2f(1,1), Point2f(300,200)};
	std::vector<Point2f> out;
	perspectiveTransform(points_in_bird_view, out, b_inv_mat);
	*/



	int coords_part [NUM_PART+1];
	sub_partition(0, image.rows, NUM_PART, true, coords_part);
	for(auto c:coords_part)
		std::cout << c << std::endl;

	cvtColor(bird, processed, COLOR_BGR2GRAY);

	show_image("roi", processed, true);
	//h_sobel(processed);
	//show_image("sobel", processed, true);
	canny_blur(processed);
	show_image("canny", processed, true);
	std::vector<Point2f> w_left_points;
	std::vector<Point2f> w_right_points;
	multiple_windows_search(processed, W_NUM_WINDOWS, W_WIDTH, w_left_points, w_right_points);
	for(auto &p:w_left_points)
	{
		//line(processed, p, p, Scalar(128), 5);
	}
	for(auto &p:w_right_points)
	{
		//line(processed, p, p, Scalar(128), 5);
	}
	show_image("multiple", processed, true);
	std::vector<Vec2f> lines;
	int histo_points[2];
	h_histogram(processed, histo_points);
	//line(processed, Point(histo_points[0], 100), Point(histo_points[0], 100), Scalar(255), 5, CV_AA);
	//line(processed, Point(histo_points[1], 100), Point(histo_points[1], 100), Scalar(255), 5, CV_AA);
	//show_image("histo", processed, true);
	window_search(processed, histo_points, W_WIDTH, w_left_points, w_right_points);
	/*for (auto p : w_left_points)
		line(processed, p, p, Scalar(255), 5, CV_AA);
	for (auto p : w_right_points)
		line(processed, p, p, Scalar(255), 5, CV_AA);
	show_image("w", processed, true);
	*/
	//draw_curve(processed, bez_points, 0);
	//draw_curve(processed, bez_points, 3);

	std::cout << "hough" << std::endl;
	//HoughLinesCustom(processed, 1, CV_PI / 180., 10, lines, NUM_LINES, 0, processed.rows);
	std::vector<Vec2f> h_left_lines;
	std::vector<Vec2f> h_right_lines;
	partitioned_hough(processed, coords_part, NUM_PART, NUM_LINES, h_left_lines, h_right_lines);
	std::cout << "hough_end" << std::endl;
	std::cout << "num_lines soll: " << NUM_PART*NUM_LINES*2 << ", num_lines ist: " << h_left_lines.size() << " + " << h_right_lines.size() << std::endl; 
	std::vector<Point2f> h_left_points;
	std::vector<Point2f> h_right_points;
	get_points(h_left_lines, h_right_lines, NUM_LINES, coords_part, h_left_points, h_right_points);

	for (unsigned int i = 0; i<h_left_points.size();++i)
	{
		line(processed, h_left_points[i], h_left_points[i], Scalar(255), 7, CV_AA);
	}
	show_image("l points", processed, true);
	for (unsigned int i = 0; i<h_right_points.size();++i)
	{
		line(processed, h_right_points[i], h_right_points[i], Scalar(255), 7, CV_AA);
	}
	show_image("r points", processed, true);

	for(auto p = h_left_points.begin(); p != h_left_points.end(); p+=2)		
		line(processed, *p, *(p+1), Scalar(125), 5, CV_AA);
	show_image("l lines", processed, true);
	for(auto p = h_right_points.begin(); p != h_right_points.end(); p+=2)		
		line(processed, *p, *(p+1), Scalar(125), 5, CV_AA);
	show_image("r lines", processed, true);

	std::cout << "end" << std::endl;

	//gabor(processed);
#ifndef NDEBUG
	//show_image(image_name, image, false);
	//show_image("Gray image", processed, true);
	/*int bla = 7;
	int part [bla+1];
	sub_partition(0, image.size[0], bla, false, part);
	for(int i = 0;i<bla+1;++i){
		std::cout << part[i] << std::endl;
		line(image, Point(0,part[i]), Point(image.size[1], part[i]), Scalar(255,0,255), 5);
	}
	show_image(image_name, image, true);
	*/

#endif
	return 0;
}