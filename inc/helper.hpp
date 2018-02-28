#pragma once
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <string>



class ParameterReader {
    private:
        std::map<std::string, std::string> data;
    public:
        // Read parameters from inputfile and save to data
        void read(const std::string &filename);
        // Get data of parameter of correct type for given key
        template <typename T> T get_value(const std::string &key);
};

/**
 * Returns the value with the correct type for a given key
 * @param key Parameter wanted
 * @return Appropriatly typed value for the given key
 */
template <typename T>
T ParameterReader::get_value(const std::string &key) {
    if (data.count(key) == 0) {
        std::cerr << "Key: " << key << ", does not exist." << std::endl;
        return "-1";
    } else {
        T param;
        std::istringstream ss(data.at(key));
        ss >> param;
        return param;
    }
}



/**
 * Makes sure that every directory ends with an '/'
 * @note makes concatenation of multiple directories or
 * @note a directory and a filename easier
 * @param old_dir A '/' is appended to it if necessary
 * @return Direcotry path with an trailing '/'
 */
template <typename T>
std::string modify_dir(T old_dir)
{
    std::string dir (old_dir);
    if (dir.back() != '/')
	{
	    return dir + "/";
	}
    return dir; 
}