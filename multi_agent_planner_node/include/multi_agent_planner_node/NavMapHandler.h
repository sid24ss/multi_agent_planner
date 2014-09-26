#pragma once
#include <multi_agent_planner/LoggerNames.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
// #include <boost/thread/mutex.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <unordered_map>
#include <string>

namespace multi_agent_planner_node {
    class NavMapHandler{
    public:
        NavMapHandler(ros::NodeHandle nh);
        void addCostmap(std::string name);
        void getInflatedMap(std::string name, std::vector<unsigned char>& data);
        void loadNavMap(const nav_msgs::OccupancyGridPtr& map);
        void setOccupancyDims(int dimx, int dimy) { dimX = dimx; dimY = dimy; }
    private:
        void crop2DMap(const std::vector<unsigned char>& v,
                       double new_origin_x, double new_origin_y,
                       double width, double height,
                       std::vector<unsigned char>& cropped_map);
        ros::NodeHandle m_nodehandle;
        std::vector< std::unique_ptr<costmap_2d::Costmap2DROS> > m_costmaps_ros;
        std::unordered_map <std::string, int> m_mapping;
        tf::TransformListener m_tf;
        nav_msgs::MapMetaData m_map_info;
        std::vector<std::unique_ptr<costmap_2d::Costmap2DPublisher> > m_costmap_publishers;
        int dimX, dimY;
    };
}   // end namepsace multi_agent_planner_node