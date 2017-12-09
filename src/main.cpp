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
	int lowThreshold = 50;
	int kernel_size = 3;
	blur(image, image, Size(3, 3));
	Canny(image, image, lowThreshold, lowThreshold * 3, kernel_size);
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

int main(int argc, char **argv)
{
	char *image_name = argv[1];
	Mat image, processed;
	image = imread(image_name, 1);
	if (argc != 2 || !image.data)
	{
		printf(" No image data \n ");
		return -1;
	}
	cvtColor(image, processed, COLOR_BGR2GRAY);
	canny_blur(processed);
	show_image("gray", processed, true);
	std::vector<Vec2f> lines;
	HoughLinesCustom(processed, 10, CV_PI/180., 100, lines, 2, 0, CV_PI);
	show_image("hough", processed, true);
	for (auto p : lines)
	{
		std::cout << p[0] << ", " << p[1] << std::endl;
		float rho = p[0], theta = p[1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(image, pt1, pt2, Scalar(255, 0, 255), 3, CV_AA);
	}
	show_image("lines", image, true);

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