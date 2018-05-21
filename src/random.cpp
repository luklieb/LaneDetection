#include "algos.hpp"

using namespace cv;



void sub_partition(const int, const int, const int, const bool, int *);
int pair_conversion(std::vector<Point2f> &, std::vector<Point2f> &);



struct r_line{
	int s_l;
	int e_l;
	double slope_l;
	int s_r;
	int e_r;
	double slope_r;
};


int random_search(Mat &img, const int num_lines, const double roi, const int num_part, const bool b_view, std::vector<Point2f> &left_points, std::vector<Point2f> &right_points)
{
    //0% overlap for each half
    const double image_split = 0.55;
    //range to search around current line for white pixels
    const int offset_x = 5;
    //standard deviation for normal distribution
    const double sigma = 38.;
    //mean for left side distribution (closer to middle of picture)
    const double m_l = 0.44 * img.cols;
    //mean for right side distribution (closer to middle of picture)
    const double m_r = img.cols - m_l - 1.;
    //maximum amount of pixels, that e_l-s_l respectively -(e_r-s_r) can differ
    //this excludes lines with wrong slope
    //-> most of left lines are right leaning, most of right lines are left leaning (same as road lanes)
    const int pixel_diff = 50;

   //coordinates of partition borders
    int coords_part[num_part + 1];
    sub_partition(roi * img.rows, img.rows, num_part, true, coords_part);
    //[start/end] [left/right] x values of points
    int s_l, e_l, s_r, e_r;
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

	uint8x8_t compare1 = vcreate_u8(0x0101010101010101);
	uint8x16_t compare = vcombine_u8(compare1, compare1);

    //height of current partition
    height = (coords_part[1] - coords_part[0]);
    height_inv = 1. / height;
    assert(height_inv >= 0.);

	r_line * candidates = new r_line[num_part*num_lines];

	for (int part = 0; part < num_part; ++part)
	{    
		std::default_random_engine generator;
    	std::normal_distribution<double> dist_left(m_l-part*0.04*img.cols, sigma);
    	std::normal_distribution<double> dist_right(m_r+part*0.022*img.cols, sigma);
 
		for (int l = 0; l < num_lines;)
		{
			s_l = dist_left(generator);
            e_l = dist_left(generator);
            s_r = dist_right(generator);
            e_r = dist_right(generator);
            if(abs(e_l - s_l) > pixel_diff || abs(e_r -s_r) > pixel_diff || (part!=0 && e_l > s_l)||  (part!=0 && e_r < s_r) ||s_l-offset_x < 0 || e_l-offset_x < 0 || s_r-offset_x < 0 || e_r-offset_x < 0 || s_l+offset_x > img.cols || e_l+offset_x > img.cols || s_r+offset_x > img.cols || e_r+offset_x > img.cols || s_l > image_split*img.cols || e_l > image_split*img.cols || s_r < (1.-image_split)*img.cols || e_r < (1.-image_split)*img.cols)
                continue;
			candidates[part*num_lines+l].s_l = s_l;
			candidates[part*num_lines+l].e_l = e_l;
 			candidates[part*num_lines+l].s_r = s_r;
			candidates[part*num_lines+l].e_r = e_r;	
            //slope of lines like this: x = slope*y + t
			candidates[part*num_lines+l].slope_l = height_inv * (e_l - s_l);
			candidates[part*num_lines+l].slope_r = height_inv * (e_r - s_r);
			++l;
		}
	}




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
        delete[] candidates;
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
	
	delete[] candidates;
    return LANEDET_SUCCESS;
}
