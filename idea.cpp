

This file is for random code snippets only....

void callback_1(int slider,void *data){
    const Point2f b_p2[4] = {Point2f((0.5 - ((double*)data[1])[2] - data[1][1]) * image.cols, 0),
                             Point2f((0.5 + data[1][2] + data[1][2]) * image.cols, 0),
                             Point2f((0.5 + data[1][2] + data[1][2]) * image.cols, image.rows),
                             Point2f((0.5 - data[1][2] - data[1][2]) * image.cols, image.rows)};

    std::vector<Point> tmp(b_p1, b_p1 + 4);
    int po = Mat(tmp).rows;
    std::cout << po << std::endl;
    const Point *pts = (const Point *)Mat(tmp).data;
    polylines(image, &pts, &po, 1, true, Scalar(0, 255, 0), 5);
    show_image("polylines", image, true);
    double para[3];
    void *data[2];
    data[0] = (void *)&image;
    data[1] = (void *)para;
}

//mat from Kitti calib data
double mat [3][3] = {{9.999239000000e-01, 9.837760000000e-03, -7.445048000000e-03 }, 
    {-9.869795000000e-03, 9.999421000000e-01, -4.278459000000e-03}, 
    {7.402527000000e-03, 4.351614000000e-03, 9.999631000000e-01}};
