#include <multi_agent_planner/swarmViz.h>

using namespace multi_agent_planner;

SwarmViz::SwarmViz(ros::NodeHandle nh)
    :   nh_(nh)
{
    marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
}

void SwarmViz::setReferenceFrame(std::string ref_frame) {
    reference_frame_ = ref_frame;
}

void SwarmViz::configureRobotParams(double radius, double nominal_z) {
    robot_radius_ = radius;
    nominal_z_ = nominal_z;
}

void SwarmViz::visualizeCircle(std::string ns, double x, double y, int
            hue)
{
    // visualizes a generic circle
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;
    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = ns;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = nominal_z_;
    marker.scale.x = robot_radius_*2.0;
    marker.scale.y = robot_radius_*2.0;
    marker.scale.z = 0.05;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(180.0);
    marker_publisher_.publish(marker);
}

void SwarmViz::visualizeCircles(std::string ns, std::vector<double> x,
                std::vector<double> y, std::vector<int> hues)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    for(size_t i = 0; i < x.size(); ++i)
    {
        leatherman::HSVtoRGB(&r, &g, &b, hues[i], 1.0, 1.0);
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = reference_frame_;
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = robot_radius_*2.0;
        marker.scale.y = robot_radius_*2.0;
        marker.scale.z = 0.05;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.5;
        marker.lifetime = ros::Duration(180.0);
        marker.id = i;
        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = nominal_z_;
        marker_array.markers.push_back(marker);
    }
    marker_array_publisher_.publish(marker_array);
}