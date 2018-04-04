#include "helper.hpp"

void Parameter_reader::read(const std::string &filename) {
    std::ifstream parameters(filename);
    std::string line;
    if (parameters.is_open()) {
        while (!parameters.eof()) {
            std::getline(parameters, line);
            if ((line.length() <= 1))
                continue;
            size_t pos = line.find_first_of(" ");
            size_t last = line.find_last_of(" ");
            //size_t ret = line.find_first_of("\n\r");
            data.insert(std::pair<std::string, std::string>(line.substr(0, pos), line.substr(last + 1, line.length())));
        }
    }
    parameters.close();
}

void show_image(const String image_name, const Mat &image, const bool wait, const String path)
{
    namedWindow(image_name, WINDOW_AUTOSIZE);
    imshow(image_name, image);
    if (wait)
        waitKey(0);
    else
        imwrite(path+image_name, image);
}

