#include "filters.hpp"
#include <arm_neon.h>
#include <omp.h>


void row_filter(Mat &image, const int thres, const int tau)
{
    //convert input image to gray (single channel, 8bit)
    cvtColor(image, image, COLOR_BGR2GRAY);
    //create copy of input image
    Mat cpy (image.rows, image.cols, CV_8UC1, Scalar(0));
    //Mat cpy = image.clone();
	//uchar x_curr, x_m_t, x_p_t;
    uint8x16_t v_curr, v_m_t, v_p_t, v_abs, v_mul, v_add, v_sub;
	uint8x16_t v_two = vmovq_n_u8(2);
	//uchar * const image_data = image.data;
	//uchar * const cpy_data = cpy.data;	

//#pragma omp parallel for private(v_curr, v_m_t, v_p_t, v_mul, v_add, v_abs, v_sub)
	for (int y = 0; y < cpy.rows; ++y)
    {
		uchar * cpy_row = cpy.ptr(y);
		uchar * image_row = image.ptr(y);
		for (int x = (tau<16?16:tau); x < cpy.cols-(tau<16?16:tau)-16; x+=16)
        {
            /*
			//current pixel value at (y,x)
            x_curr = cpy.at<uchar>(y, x);
			//avoid boundary layers
            //get pixel value from tau offset
            x_m_t = cpy.at<uchar>(y, x - tau);
            x_p_t = cpy.at<uchar>(y, x + tau);
            //avoid overflow of 8 bit type
            //compute new value for pixel from offset values
            image.at<uchar>(y, x) = saturate_cast<uchar>(2 * x_curr - (x_m_t + x_p_t) - abs(x_m_t - x_p_t));
        	*/
			v_curr = vld1q_u8(image_row + x);
			v_m_t = vld1q_u8(image_row + x - tau);
			v_p_t = vld1q_u8(image_row + x + tau);
			v_mul = vmulq_u8(v_two, v_curr);
			v_add = vaddq_u8(v_m_t, v_p_t);
 		    v_abs = vabdq_u8(v_m_t, v_p_t);
			v_sub = vsubq_u8(v_mul, v_add);
			vst1q_u8(cpy_row + x, vsubq_u8(v_sub, v_abs));
		}       
    }
	image = cpy;
    threshold(image, image, thres, 255, THRESH_BINARY);
}


