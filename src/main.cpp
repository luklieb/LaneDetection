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

	//#############################################################################################
	//######################################### Random ############################################
	//#############################################################################################

	//int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
	//double fontScale = 2;
	//int thickness = 3;
	//putText(processed, std::to_string(number), p, fontFace, fontScale, Scalar::all(255), thickness, 8);

	//#############################################################################################
	//######################################### Parameter File ####################################
	//#############################################################################################
	const double ROI_START = 0.6;
	const int NUM_PART = 5;
	const int NUM_LINES = 3;
	const double B_OFFSET_MID = 0.04;
	const double B_OFFSET = 0.4;
	const double B_offset2 = 0.05;
	const double B_HEIGHT = ROI_START; //or: ROI_START
	const int W_NUM_WINDOWS = 10;
	const int W_WIDTH = 40;

	//#############################################################################################
	//######################################### Program ###########################################
	//#############################################################################################

	const Point2f b_p1[4] = {Point2f((0.5 - B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID + B_OFFSET) * image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_OFFSET) * image.cols, image.rows)};
	const Point2f b_p2[4] = {Point2f((0.5 - B_OFFSET_MID - B_offset2) * image.cols, 0), Point2f((0.5 + B_OFFSET_MID + B_offset2) * image.cols, 0), Point2f((0.5 + B_OFFSET_MID + B_offset2) * image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_offset2) * image.cols, image.rows)};
	const Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
	const Mat b_inv_mat = getPerspectiveTransform(b_p2, b_p1);
	warpPerspective(image, bird, b_mat, Size(image.cols, image.rows));
	show_image("bird", bird, true);

	/* //IMPORTANT!!!
	std::vector<Point2f> points_in_bird_view = {Point2f(1,1), Point2f(300,200)};
	std::vector<Point2f> out;
	perspectiveTransform(points_in_bird_view, out, b_inv_mat);
	*/

	int coords_part[NUM_PART + 1];
	sub_partition(0, image.rows, NUM_PART, true, coords_part);
	for (auto c : coords_part)
		std::cout << c << std::endl;

	cvtColor(bird, processed, COLOR_BGR2GRAY);

	show_image("roi", processed, true);
	//h_sobel(processed);
	//show_image("sobel", processed, true);
	canny_blur(processed);
	show_image("canny", processed, true);
	std::vector<Point2f> w_left_points;
	std::vector<Point2f> w_right_points;
	std::cout << "multiple" << std::endl;
	multiple_windows_search(processed, W_NUM_WINDOWS, W_WIDTH, w_left_points, w_right_points);
	std::cout << "multiple done" << std::endl;
	for (auto &p : w_left_points)
	{
		//line(processed, p, p, Scalar(128), 5);
	}
	for (auto &p : w_right_points)
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
	std::cout << "num_lines soll: " << NUM_PART * NUM_LINES * 2 << ", num_lines ist: " << h_left_lines.size() << " + " << h_right_lines.size() << std::endl;
	std::vector<Point2f> h_left_points;
	std::vector<Point2f> h_right_points;
	get_points(h_left_lines, h_right_lines, NUM_LINES, coords_part, h_left_points, h_right_points);
	Mat tmp = processed.clone();
	for (unsigned int i = 0; i < h_left_points.size(); ++i)
	{
		//line(tmp, h_left_points[i], h_left_points[i], Scalar(255), 7, CV_AA);
	}
	//show_image("tmp l points", tmp, true);
	for (unsigned int i = 0; i < h_right_points.size(); ++i)
	{
		//line(tmp, h_right_points[i], h_right_points[i], Scalar(255), 7, CV_AA);
	}
	show_image("tmp r points", tmp, true);

	std::cout << "alm" << std::endl;
	alm(h_left_points, h_right_points, NUM_PART, NUM_LINES);
	std::cout << "alm done" << std::endl;
	std::cout << "alm lines is: " << h_right_points.size() << ", should: " << 2 * NUM_PART << std::endl;

	for (unsigned int i = 0; i < h_left_points.size(); ++i)
	{
		line(processed, h_left_points[i], h_left_points[i], Scalar(255), 7, CV_AA);
	}
	show_image("alm l points", processed, true);

	for (unsigned int i = 0; i < h_right_points.size(); ++i)
	{
		line(processed, h_right_points[i], h_right_points[i], Scalar(255), 7, CV_AA);
	}
	show_image("alm r points", processed, true);

	for (auto p = h_left_points.begin(); p != h_left_points.end(); p += 2)
		line(processed, *p, *(p + 1), Scalar(125), 5, CV_AA);
	show_image("alm l lines", processed, true);
	for (auto p = h_right_points.begin(); p != h_right_points.end(); p += 2)
		line(processed, *p, *(p + 1), Scalar(125), 5, CV_AA);
	show_image("alm r lines", processed, true);

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