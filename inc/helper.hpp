#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

class ParameterReader {
    private:
        std::map<std::string, std::string> data;
    public:
        // Read parameters from inputfile and save to data
        void read(const std::string &filename);
        // Get data of parameter of correct type for given key
        template <typename T> T get_value(const std::string &key);
};