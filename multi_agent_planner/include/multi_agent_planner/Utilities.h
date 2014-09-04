#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

namespace multi_agent_planner {

/**
 * @brief combines two vectors
 * @details combines the vector v1 into v2. That is, adds v1 to the end of v2
 * 
 * @param v2 the final vector you want
 * @tparam T the type of vectors you want to combine
 */
template <typename T>
void combineVectors(std::vector<T>&& v1, std::vector<T>& v2)
{
    for (auto& v : v1) {
        v2.push_back(v);
    }
}

template <typename T>
void combineVectors(const std::vector<T>& v1, std::vector<T>& v2)
{
    combineVectors(std::move(v1), v2);
}

/**
 * @brief convert a vector into a string
 * @details useful for printing vectors out in a single line
 * 
 * @param v the vector to convert to a string
 * @tparam T the type of the vector
 * @return a std::string representation of the vector, separated by spaces.
 */
template <typename T>
std::string vectorToString(const std::vector<T>& v) {
    return vectorToString(std::move(v));
}

template <typename T>
std::string vectorToString(std::vector<T>&& v) {
    std::stringstream ss;
    for (auto& val : v) {
        ss << val << " ";
    }
    return ss.str();
}

}