#include "calibration.hpp"



//p1 (low,low), p2(high,low), p3(high,high), p4(low,high) => order of points
static void draw_trapezoid(Mat &img, const Point2f &p1, const Point2f &p2, const Point2f &p3, const Point2f &p4)
{
    line(img, p1, p2, Scalar(255));
    line(img, p2, p3, Scalar(255));
    line(img, p3, p4, Scalar(255));
    line(img, p1, p4, Scalar(255));
}

//offset_mid = 0
static void callback_0(int slider, void *data)
{
    calib_data *d_ptr = (calib_data *)data;
    calib_data d = *d_ptr;
    Mat dst = (*(d.img)).clone();
    Mat b_view;
    const Point2f b_p1[4] = {Point2f((0.5 - 0.01 * slider) * (*(d.img)).cols, d.array[3] * (*(d.img)).rows),
                             Point2f((0.5 + 0.01 * slider) * (*(d.img)).cols, d.array[3] * (*(d.img)).rows),
                             Point2f((0.5 + 0.01 * slider + d.array[1]) * (*(d.img)).cols, (*(d.img)).rows),
                             Point2f((0.5 - 0.01 * slider - d.array[1]) * (*(d.img)).cols, (*(d.img)).rows)};
    const Point2f b_p2[4] = {Point2f((0.5 - 0.01 * slider - d.array[2]) * (*(d.img)).cols, 0),
                             Point2f((0.5 + 0.01 * slider + d.array[2]) * (*(d.img)).cols, 0),
                             Point2f((0.5 + 0.01 * slider + d.array[2]) * (*(d.img)).cols, (*(d.img)).rows),
                             Point2f((0.5 - 0.01 * slider - d.array[2]) * (*(d.img)).cols, (*(d.img)).rows)};
    //normalize to 1
    d.array[0] = 0.01 * slider;

    draw_trapezoid(dst, b_p1[0], b_p1[1], b_p1[2], b_p1[3]);
    draw_trapezoid(dst, b_p2[0], b_p2[1], b_p2[2], b_p2[3]);

    imshow("Calibration", dst);
    Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
    warpPerspective(dst, b_view, b_mat, Size(dst.cols, dst.rows));
    imshow("Birdview", b_view);
    std::cout << "offset_mid: " << d.array[0] << ", offset: " << d.array[1] << ", offset2: " << d.array[2] << ", height: " << d.array[3] << std::endl;
}

//offset = 1
static void callback_1(int slider, void *data)
{
    calib_data *d_ptr = (calib_data *)data;
    calib_data d = *d_ptr;
    Mat img = *(d.img);
    Mat dst = img.clone();
    Mat b_view;
    const Point2f b_p1[4] = {Point2f((0.5 - d.array[0]) * img.cols, d.array[3] * img.rows),
                             Point2f((0.5 + d.array[0]) * img.cols, d.array[3] * img.rows),
                             Point2f((0.5 + d.array[0] + 0.01 * slider) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - 0.01 * slider) * img.cols, img.rows)};
    const Point2f b_p2[4] = {Point2f((0.5 - d.array[0] - d.array[2]) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + d.array[2]) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + d.array[2]) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - d.array[2]) * img.cols, img.rows)};
    d.array[1] = 0.01 * slider;

    draw_trapezoid(dst, b_p1[0], b_p1[1], b_p1[2], b_p1[3]);
    draw_trapezoid(dst, b_p2[0], b_p2[1], b_p2[2], b_p2[3]);
    imshow("Calibration", dst);
    Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
    warpPerspective(dst, b_view, b_mat, Size(dst.cols, dst.rows));
    imshow("Birdview", b_view);
    std::cout << "offset_mid: " << d.array[0] << ", offset: " << d.array[1] << ", offset2: " << d.array[2] << ", height: " << d.array[3] << std::endl;
}

//offset2 = 2
static void callback_2(int slider, void *data)
{
    calib_data *d_ptr = (calib_data *)data;
    calib_data d = *d_ptr;
    Mat img = *(d.img);
    Mat dst = img.clone();
    Mat b_view;
    const Point2f b_p1[4] = {Point2f((0.5 - d.array[0]) * img.cols, d.array[3] * img.rows),
                             Point2f((0.5 + d.array[0]) * img.cols, d.array[3] * img.rows),
                             Point2f((0.5 + d.array[0] + d.array[1]) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - d.array[1]) * img.cols, img.rows)};
    const Point2f b_p2[4] = {Point2f((0.5 - d.array[0] - 0.01 * slider) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + 0.01 * slider) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + 0.01 * slider) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - 0.01 * slider) * img.cols, img.rows)};
    d.array[2] = 0.01 * slider;

    draw_trapezoid(dst, b_p1[0], b_p1[1], b_p1[2], b_p1[3]);
    draw_trapezoid(dst, b_p2[0], b_p2[1], b_p2[2], b_p2[3]);

    imshow("Calibration", dst);
    Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
    warpPerspective(dst, b_view, b_mat, Size(dst.cols, dst.rows));
    imshow("Birdview", b_view);
    std::cout << "offset_mid: " << d.array[0] << ", offset: " << d.array[1] << ", offset2: " << d.array[2] << ", height: " << d.array[3] << std::endl;
}

//height = 3
static void callback_3(int slider, void *data)
{
    calib_data *d_ptr = (calib_data *)data;
    calib_data d = *d_ptr;
    Mat img = *(d.img);
    Mat dst = img.clone();
    Mat b_view;
    const Point2f b_p1[4] = {Point2f((0.5 - d.array[0]) * img.cols, 0.01 * slider * img.rows),
                             Point2f((0.5 + d.array[0]) * img.cols, 0.01 * slider * img.rows),
                             Point2f((0.5 + d.array[0] + d.array[1]) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - d.array[1]) * img.cols, img.rows)};
    const Point2f b_p2[4] = {Point2f((0.5 - d.array[0] - d.array[2]) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + d.array[2]) * img.cols, 0),
                             Point2f((0.5 + d.array[0] + d.array[2]) * img.cols, img.rows),
                             Point2f((0.5 - d.array[0] - d.array[2]) * img.cols, img.rows)};
    d.array[3] = 0.01 * slider;

    draw_trapezoid(dst, b_p1[0], b_p1[1], b_p1[2], b_p1[3]);
    draw_trapezoid(dst, b_p2[0], b_p2[1], b_p2[2], b_p2[3]);

    imshow("Calibration", dst);
    Mat b_mat = getPerspectiveTransform(b_p1, b_p2);
    warpPerspective(dst, b_view, b_mat, Size(dst.cols, dst.rows));
    imshow("Birdview", b_view);
    std::cout << "offset_mid: " << d.array[0] << ", offset: " << d.array[1] << ", offset2: " << d.array[2] << ", height: " << d.array[3] << std::endl;
}

void b_view_calibration(Mat *img, double offset_mid, double offset, double offset2, double height)
{
    //offset_mid = 0, offset = 1, offset2 = 2, height = 3;
    double *param = new double[4];
    //initial trapezoids shapes; values between 0 and 1
    param[0] = offset_mid;
    param[1] = offset;
    param[2] = offset2;
    param[3] = height;
    std::cout << "start:" << std::endl;
    std::cout << "offset_mid: " << param[0] << ", offset: " << param[1] << ", offset2: " << param[2] << ", height: " << param[3] << std::endl;
    //initial trackbar positions, trackers are between 0 and 100
    int o_mid = offset_mid * 100.;
    int o = offset * 100.;
    int o2 = offset2 * 100.;
    int h = height * 100.;
    calib_data *data = new calib_data{img, param};
    namedWindow("Calibration", 1);
    namedWindow("Birdview", 1);
    createTrackbar("offset_mid", "Calibration", &o_mid, 100, callback_0, data);
    createTrackbar("offset", "Calibration", &o, 100, callback_1, data);
    createTrackbar("offset2", "Calibration", &o2, 100, callback_2, data);
    createTrackbar("height", "Calibration", &h, 100, callback_3, data);
    //27 is ESC key -> if you press and hold ESC it closes all windows
    if(waitKey(0) == 27){
        destroyWindow("Calibration");
        destroyWindow("Birdview");
    }
}