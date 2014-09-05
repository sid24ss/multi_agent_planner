#pragma once
#include <string>
#include <memory>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <leatherman/utils.h>

namespace multi_agent_planner {
    class SwarmViz {
    public:
        SwarmViz(ros::NodeHandle nh);
        void setReferenceFrame(std::string ref_frame);
        // TODO : Make this more generic so that we can visualize a footprint
        void configureRobotParams(double radius, double nominal_z);

        // visualization functions
        void visualizeCircle(std::string ns, double x, double y, int
            hue);
        void visualizeCircles(std::string ns, std::vector<double> x, std::vector<
            double> y, std::vector<int> hues);
    private:
        ros::NodeHandle nh_;
        std::string reference_frame_;
        double robot_radius_;
        double nominal_z_;
        ros::Publisher marker_array_publisher_;
        ros::Publisher marker_publisher_;
    };
}