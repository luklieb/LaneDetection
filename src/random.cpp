#include "algos.hpp"

using namespace cv;



void sub_partition(const int, const int, const int, const bool, int *);
int pair_conversion(std::vector<Point2f> &, std::vector<Point2f> &);


int random_search(Mat &img, const int num_lines, const double roi, const int num_part, const bool b_view, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points, r_line * candidates)
{
    //0% overlap for each half
    const double image_split = 0.5;
    //range to search around current line for white pixels
    const int offset_x = 5;
    
   //coordinates of partition borders
    int coords_part[num_part + 1];
    sub_partition(roi * img.rows, img.rows, num_part, true, coords_part);
    //store temporarily the best x-[start/end]-[left/right]-points
    int s_l_best = 0, e_l_best = 0, s_r_best = 0, e_r_best = 0;
    //score of line quality (sum of white pixels along the current line)
    int score_l = 0, score_r = 0;
    //store temporarily the best score ->
    int score_l_best = 0, score_r_best = 0;
    //variables for line parameter calculation
    double height_inv;
    double height;

	left_points.resize(num_part*2);
	right_points.resize(num_part*2);

	uchar* img_data = img.data;

	const uint8x8_t compare1 = vcreate_u8(0x0101010101010101);
	const uint8x16_t mask = vcombine_u8(compare1, compare1);
	uint8x16_t l_vec, r_vec, l_masked, r_masked;
	uint8_t l_result, r_result;

    //height of current partition
    height = (coords_part[1] - coords_part[0]);
    height_inv = 1. / height;
    assert(height_inv >= 0.);


    //for each partition
	for (int part = 0; part < num_part; ++part)
    {
        //num_lines on both sides and calculate for the current line the quality
        //(-> move along the line an count white pixels)
        //if current line for one side is better than the former best, temporarily store the configuration
  		for (int l = 0; l < num_lines; ++l)
        {
#ifndef NDEBUG
            line(img, Point2f(candidates[part*num_lines+l].s_l, coords_part[part]), Point2f(candidates[part*num_lines+l].e_l, coords_part[part + 1]), Scalar(128));
            line(img, Point2f(candidates[part*num_lines+l].s_r, coords_part[part]), Point2f(candidates[part*num_lines+l].e_r, coords_part[part + 1]), Scalar(128));
#endif
            //calc quality scores for both lines
            for (int y = 0; y < height; ++y)
            {
                int x_l_curr = y * candidates[part*num_lines+l].slope_l + candidates[part*num_lines+l].s_l;
                int x_r_curr = y * candidates[part*num_lines+l].slope_r + candidates[part*num_lines+l].s_r;
				/*
                for (int x_offset = -offset_x; x_offset <= (int)offset_x; ++x_offset)
                {
                    if (img.at<uchar>(y + coords_part[part], x_l_curr + x_offset) >= 250)
                    {
						++score_l;
                    }
					if (img.at<uchar>(y + coords_part[part], x_r_curr + x_offset) >= 250)
                    {    
						++score_r;
                	}
				}
				*/
				l_vec = vld1q_u8(img_data + (y+coords_part[part])*img.cols + x_l_curr-8);
				r_vec = vld1q_u8(img_data + (y+coords_part[part])*img.cols + x_r_curr-8);
				l_masked = vandq_u8(l_vec, mask);
				r_masked = vandq_u8(r_vec, mask);
				score_l += vaddvq_u8(l_masked);
				score_r += vaddvq_u8(r_masked);
				//std::cout << "score_l: " << score_l << ", score_r: " << score_r << std::endl;
			
			}
            //store current configuration for both sides
            //if it is better than the all time best
            if (score_l > score_l_best)
            {
				score_l_best = score_l;
                s_l_best = candidates[part*num_lines+l].s_l;
                e_l_best = candidates[part*num_lines+l].e_l;
            }
            if (score_r > score_r_best)
            {
                score_r_best = score_r;
                s_r_best = candidates[part*num_lines+l].s_r;
                e_r_best = candidates[part*num_lines+l].e_r;
            }
            score_l = 0;
            score_r = 0;
        }
		left_points[2*part] = Point2f(s_l_best, coords_part[part]);
		left_points[2*part+1] = Point2f(e_l_best, coords_part[part+1]);
		right_points[2*part] = Point2f(s_r_best, coords_part[part]);
		right_points[2*part+1] = Point2f(e_r_best, coords_part[part+1]);

		score_l_best = 0;
        score_r_best = 0;
    }
    
	
	if (left_points.size() != num_part * 2u || right_points.size() != num_part * 2u)
    {
		std::cout << "warning in " << __FUNCTION__ << ", line: " << __LINE__ << std::endl;
        return LANEDET_WARNING;
    }

#ifndef NDEBUG
    show_image("all random", img, true);
    for (auto p = left_points.begin(); p != left_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 4);
    for (auto p = right_points.begin(); p != right_points.end(); p += 2)
        line(img, *p, *(p + 1), Scalar(255), 4);
    std::cout << "num points after random(): " << left_points.size() << ", right: " << right_points.size() << std::endl;
    show_image("random finished", img, true);
#endif

    //Compute and return the mean of the two points for each side with same y-coordinate (on partition boundary)
    pair_conversion(left_points, right_points);
	
    return LANEDET_SUCCESS;
}
