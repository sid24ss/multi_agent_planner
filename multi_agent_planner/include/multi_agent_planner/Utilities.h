#pragma once
#include <utility>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

namespace multi_agent_planner {

/**
 * @brief combines two vectors
 * @details combines the vector v1 into v2. That is, adds v1 to the end of v2
 * 
 * @param v2 the final vector you want
 * @tparam T the type of vectors you want to combine
 */
template <typename T>
void combineVectors(const std::vector<T>& v1, std::vector<T>& v2)
{
    for (auto& v : v1) {
        v2.push_back(v);
    }
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
    std::stringstream ss;
    for (auto& val : v) {
        ss << val << " ";
    }
    return ss.str();
}

/**
 * @brief signum function
 * @details returns the sign of the argument
 * 
 * @param l the value that you want the sign of
 * @tparam T the type of the value
 * @return -1 if negative and +1 if positive; 0 if 0
 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief returns the euclidean norm of the vector
 * @details the norm is sqrt(v_1**2 + v_2**2 + ... )
 * 
 * @param v the vector you want the norm for
 * @tparam T the type of the vector
 * @return the euclidean norm
 */
template <typename T>
double vectorNorm(const std::vector<T>& v)
{
    double norm = 0.0;
    for (auto& e : v)
        norm += e*e;
    return std::sqrt(norm);
}

/**
 * @brief generate a random double value within range
 * 
 * @param lower the lower bound for the value
 * @param higher the upper bound for the value
 * 
 * @return random double
 */
inline double randomDouble(const double lower, const double higher)
{
    return lower + (higher-lower) * ( double(rand()) / RAND_MAX );
}

/**
 * @brief discretizes a line
 * @details gets the bresenham line points between two points
 * 
 * @param x1 start point x
 * @param y1 start point y
 * @param x2 end point x
 * @param y2 end point y
 * @param pts_x output x coordinates
 * @param pts_y output y coordinates
 */
inline std::vector< std::pair<int,int> > getBresenhamLinePoints(int x1, int y1, int x2, int y2)
{
    std::vector <std::pair<int, int> > out_points;
    bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));
    if(steep){
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2){
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    int dx = x2 - x1;
    int dy = std::abs(y2 - y1);

    double err = dx / 2.0f;
    
    int ystep = (y1 < y2)? 1 : -1;
    int y = (y1);

    for (int x = x1; x <= x2; ++x) {
        if(steep){
            // y,x
            out_points.push_back(std::make_pair(y,x));
        } else {
            // x, y
            out_points.push_back(std::make_pair(x, y));
        }
        err = err - dy;
        if(err < 0){
            y = y + ystep;
            err = err +  dx;
        }
    }
    return out_points;
}


}   // close namespace multi_agent_planner