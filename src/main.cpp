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
 * @param img the image for the Hough Transformation
 * @param part_coords holds the num_part+1 coordinates of the partitions
 * @param num_part number of sub-domains/partitions
 * @param num_lines number of lines per partition per side 
 * @param lines vector holding the polar coordinates of the detected lines
 * @note lines stores the detected lines the following way: beginning of vector
 * 		the left num_lines lines of the first partition, the right num_lines of the first partition, ...
 * 		..., the right num_lines of the last partition at the end of the vector
 * 		--> for i=0...num_part-1: append( num_lines*left(partition_i), num_lines*right(partition_i) )
 * @note lines has size 2*num_lines*num_part 
 */
void partitioned_hough(const Mat &img, const int *part_coords, const int num_part, const int num_lines, std::vector<Vec2f> &lines)
{
	std::vector<Vec2f> lines_tmp;
	for(int i = 0; i<num_part; ++i)
	{
		HoughLinesCustom(img, 1., CV_PI / 180., 10, lines_tmp, num_lines, part_coords[i], part_coords[i+1]);
		lines.insert(lines.end(), lines_tmp.begin(), lines_tmp.end());
		lines_tmp.clear();
	}
}

/**
 * Converts the polar coordiantes from a Hough Transform to points according to their start/end of each partition
 * @param lines The polar coordinates returned from a Hough Transform
 * @param num_lines The number of lines of each side of each partition
 * @param coords_part Coordinates of the partitions
 * @param points Holds the converted Points 
 */
void get_points(std::vector<Vec2f> &lines, const int num_lines, const int *coords_part, std::vector<Point> &points)
{
	int i = 0;
	int j = 0;
	for (auto p : lines)
	{
		//std::cout << p[0] << ", " << p[1] / CV_PI << std::endl;
		float rho = p[0], theta = p[1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta), a_inv = 1./a;;
		pt1.y = coords_part[i];
		pt1.x = (rho - pt1.y*b)*a_inv;
		pt2.y = coords_part[i+1];	
		pt2.x = (rho - pt2.y*b)*a_inv;
		points.push_back(pt1);
		points.push_back(pt2);
		++j;
		if(j%(num_lines*2) == 0) //num_lines for each left and right half
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
	 * To be read in from parameter file
	 */
	const double ROI_START = 0.3;
	const int NUM_PART = 2;
	const int NUM_LINES = 1;
	const double B_OFFSET_MID = 0.04;
	const double B_OFFSET = 0.4;
    const double B_offset2 = 0.05;
    const double B_HEIGHT = 0.6; //or: ROI_START

	/**
	 * Program
	 */

	const Point2f b_p1[4] = {Point2f((0.5-B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID + B_OFFSET)*image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_OFFSET)*image.cols, image.rows)};
    const Point2f b_p2[4] = {Point2f((0.5-B_OFFSET_MID-B_offset2)*image.cols, 0), Point2f((0.5+B_OFFSET_MID+B_offset2)*image.cols, 0), Point2f((0.5+B_OFFSET_MID+B_offset2)*image.cols, image.rows), Point2f((0.5-B_OFFSET_MID-B_offset2)*image.cols, image.rows)};

	const Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
	const Mat b_inv_mat = getPerspectiveTransform(b_p2, b_p1);
	warpPerspective(image, bird, b_mat, Size(image.cols, image.rows));
	show_image("bird", bird, true);
	line(mask, Point(1, 1), Point(300,200), Scalar(200,0,200), 20, 8, 0);
	std::vector<Point2f> poi = {Point2f(1,1), Point2f(300,200)};
	std::vector<Point2f> out;
	perspectiveTransform(poi, out, b_inv_mat);

	line(image, out[0], out[1], Scalar(0,200,0), 1, 8, 0);
	show_image("transfrom", image, true);

	show_image("line", mask, true);
	warpPerspective(mask, mask, b_inv_mat, Size(image.cols, image.rows));
	show_image("trans mask", mask, true);
	mask.copyTo(image,mask);
	show_image("orig mit", image, true);



	int coords_part [NUM_PART+1];
	sub_partition(image.rows*ROI_START, image.rows, NUM_PART, true, coords_part);
	for(auto c:coords_part)
		std::cout << c << std::endl;

	cvtColor(bird, processed, COLOR_BGR2GRAY);
	//canny_blur(processed);
	show_image("roi", processed, true);
	//h_sobel(processed);
	//show_image("sobel", processed, true);
	canny_blur(processed);
	show_image("canny", processed, true);
	std::vector<Vec2f> lines;
	int histo_points[2];
	h_histogram(processed, histo_points);
	line(processed, Point(histo_points[0], 100), Point(histo_points[0], 100), Scalar(255), 5, CV_AA);
	line(processed, Point(histo_points[1], 100), Point(histo_points[1], 100), Scalar(255), 5, CV_AA);
	//v_roi(processed, ROI_START);
	show_image("roi", processed, true);
	std::vector<Point> bez_points = std::vector<Point>(6);
	window_search(processed, histo_points, 30, bez_points);
	for (auto p : bez_points)
		line(processed, p, p, Scalar(255), 5, CV_AA);
	show_image("window", processed, true);

	draw_curve(processed, bez_points, 0);
	draw_curve(processed, bez_points, 3);

	std::cout << "hough" << std::endl;
	//HoughLinesCustom(processed, 1, CV_PI / 180., 10, lines, NUM_LINES, 0, processed.rows);
	partitioned_hough(processed, coords_part, NUM_PART, NUM_LINES, lines);
	std::cout << "hough_end" << std::endl;
	std::cout << "num_lines soll: " << NUM_PART*NUM_LINES*2 << ", num_lines ist: " << lines.size() << std::endl; 
	std::vector<Point> points;
	get_points(lines, NUM_LINES, coords_part, points);

	for (unsigned int i = 0; i<points.size();++i)
	{
		line(processed, points[i], points[i], Scalar((double)i/points.size()*255., 255), 7, CV_AA);
	}
	show_image("points", processed, true);


	for(auto p = points.begin(); p != points.end(); p+=2)		
		line(processed, *p, *(p+1), Scalar(125), 5, CV_AA);

	std::cout << "draw" << std::endl;
	show_image("lines", processed, true);
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