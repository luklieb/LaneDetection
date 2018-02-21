#include "helper.hpp"


void ParameterReader::read(const std::string &filename) {
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

template <typename T>
T ParameterReader::get_value(const std::string &key) {
    if (data.count(key) == 0) {
        std::cerr << "Key: " << key << ",does not exist." << std::endl;
        return "-1";
    } else {
        T param;
        std::istringstream ss(data.at(key));
        ss >> param;
        return param;
    }
}



