#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "algos.hpp"

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
		printf("not right amount of args\n Call like this: ./mapra <input_directory_path> <input_file_name> <result_directory_path");
		return 1;
	}
	String input_dir = argv[1];
	String input_file = argv[2];
	String result_dir = argv[3];
	String image_name = input_dir + input_file;
	Mat image, processed, bird;
	image = imread(image_name, 1);
	Mat mask = Mat(Size(image.cols, image.rows), image.type());
	if (!image.data)
	{
		printf("No image data\n");
		return 1;
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
	const int NUM_PART = 2;
	const int NUM_LINES = 3;
	const bool B_VIEW = false;
	int P_START;
	if (B_VIEW)
	{
		P_START = 0;
		//TODO add canny edge parameters
	}
	else
	{
		P_START = ROI_START * image.rows;
	}
	const double B_OFFSET_MID = 0.04;
	const double B_OFFSET = 0.4;
	const double B_offset2 = 0.05;
	const double B_HEIGHT = ROI_START; //or: ROI_START
	const int W_NUM_WINDOWS = 10;
	const int W_WIDTH = 40;
	const int ORDER = 1; //or NUM_PART-1

	//#############################################################################################
	//######################################### Program ###########################################
	//#############################################################################################

	if (B_VIEW)
	{
		const Point2f b_p1[4] = {Point2f((0.5 - B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID) * image.cols, B_HEIGHT * image.rows), Point2f((0.5 + B_OFFSET_MID + B_OFFSET) * image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_OFFSET) * image.cols, image.rows)};
		const Point2f b_p2[4] = {Point2f((0.5 - B_OFFSET_MID - B_offset2) * image.cols, 0), Point2f((0.5 + B_OFFSET_MID + B_offset2) * image.cols, 0), Point2f((0.5 + B_OFFSET_MID + B_offset2) * image.cols, image.rows), Point2f((0.5 - B_OFFSET_MID - B_offset2) * image.cols, image.rows)};
		const Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
		const Mat b_inv_mat = getPerspectiveTransform(b_p2, b_p1);
		warpPerspective(image, processed, b_mat, Size(image.cols, image.rows));
		show_image("bird", processed, true);
		cvtColor(processed, processed, COLOR_BGR2GRAY);
	}
	else
	{
		cvtColor(image, processed, COLOR_BGR2GRAY);
	}
	/* //IMPORTANT Bird View !!!
	std::vector<Point2f> points_in_bird_view = {Point2f(1,1), Point2f(300,200)};
	std::vector<Point2f> out;
	perspectiveTransform(points_in_bird_view, out, b_inv_mat);
	*/

	int coords_part[NUM_PART + 1];
	sub_partition(P_START, image.rows, NUM_PART, true, coords_part);
	for (auto c : coords_part)
	{
		std::cout << c << std::endl;
		line(processed, Point(50, c), Point(50, c), Scalar(255), 5);
	}

	show_image("roi", processed, true);
	//h_sobel(processed);
	//show_image("sobel", processed, true);
	canny_blur(processed);
	show_image("canny", processed, true);
	std::vector<Point2f> w_left_points;
	std::vector<Point2f> w_right_points;
	/*
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
	*/
	/*
	for (auto p : w_left_points)
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
	partitioned_hough(processed, coords_part, NUM_PART, NUM_LINES, h_left_lines, h_right_lines, B_VIEW);
	std::cout << "hough_end" << std::endl;
	std::cout << "num_lines soll: " << NUM_PART * NUM_LINES * 2 << ", num_lines ist: " << h_left_lines.size() << " + " << h_right_lines.size() << std::endl;
	std::vector<Point2f> h_left_points;
	std::vector<Point2f> h_right_points;
	get_points(h_left_lines, h_right_lines, NUM_LINES, coords_part, h_left_points, h_right_points);
	Mat tmp = processed.clone();
	std::cout << "hough left:" << std::endl;
	for (unsigned int i = 0; i < h_left_points.size(); ++i)
	{
		line(tmp, h_left_points[i], h_left_points[i], Scalar(255), 7, CV_AA);
		std::cout << h_left_points[i];
		if ((i + 1) % (NUM_LINES * 2) == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;
	show_image("tmp l points", tmp, true);
	std::cout << "hough right:" << std::endl;
	for (unsigned int i = 0; i < h_right_points.size(); ++i)
	{
		line(tmp, h_right_points[i], h_right_points[i], Scalar(255), 7, CV_AA);
		std::cout << h_right_points[i];
		if ((i + 1) % (NUM_LINES * 2) == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;
	show_image("tmp r points", tmp, true);

	std::cout << "alm" << std::endl;
	alm(h_left_points, h_right_points, NUM_PART, NUM_LINES);
	std::cout << "alm done" << std::endl;
	std::cout << "alm lines is: " << h_right_points.size() << ", should: " << 2 * NUM_PART << std::endl;

	for (unsigned int i = 0; i < h_left_points.size(); ++i)
	{
		line(processed, h_left_points[i], h_left_points[i], Scalar(128), 7, CV_AA);
	}
	show_image("alm l points", processed, true);

	for (unsigned int i = 0; i < h_right_points.size(); ++i)
	{
		line(processed, h_right_points[i], h_right_points[i], Scalar(128), 7, CV_AA);
	}
	show_image("alm r points", processed, true);

	/*for (auto p = h_left_points.begin(); p != h_left_points.end(); p += 2)
		line(processed, *p, *(p + 1), Scalar(125), 5, CV_AA);
	show_image("alm l lines", processed, true);
	for (auto p = h_right_points.begin(); p != h_right_points.end(); p += 2)
		line(processed, *p, *(p + 1), Scalar(125), 5, CV_AA);
	show_image("alm r lines", processed, true);
	*/

	alm_conversion(h_left_points, h_right_points);
	for (unsigned int i = 0; i < h_left_points.size(); ++i)
	{
		line(processed, h_left_points[i], h_left_points[i], Scalar(255), 5, CV_AA);
	}
	show_image("conveted alm l points", processed, true);

	for (unsigned int i = 0; i < h_right_points.size(); ++i)
	{
		line(processed, h_right_points[i], h_right_points[i], Scalar(255), 5, CV_AA);
	}
	show_image("converted alm r points", processed, true);

	std::cout << "left points: ";
	for (auto p : h_left_points)
	{
		std::cout << p << ", ";
	}
	std::cout << std::endl;
	std::cout << "right points: ";
	for (auto p : h_right_points)
	{
		std::cout << p << ", ";
	}
	std::cout << std::endl;

	/*h_left_points.clear();
	h_left_points.push_back(Point2f(2,4));
	h_left_points.push_back(Point2f(0,0));
	h_left_points.push_back(Point2f(-1,1));
	*/

	//TODO draw poly function wrong

	std::vector<double> right_coeff;
	std::vector<double> left_coeff;

	poly_reg(h_left_points, h_right_points, left_coeff, right_coeff, ORDER);

	std::cout << "left coeff: ";
	for (auto c : left_coeff)
	{
		std::cout << c << ", ";
	}
	std::cout << std::endl
			  << "right coeff: ";

	for (auto c : right_coeff)
	{
		std::cout << c << ", ";
	}
	std::cout << std::endl;

	draw_poly(processed, left_coeff, right_coeff, ORDER);
	show_image("drawn", processed, true);

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