#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <string>

namespace multi_agent_planner {

    struct Point3D {
        double x;
        double y;
        double z;
    };

    class StateSpaceParameters {
        // add functions to load in values
        private:
            std::map<std::string, std::pair<float, float> > bounds;
            std::map<std::string, float> resolutions;
    };

    typedef struct {
        double env_resolution;
        std::string motion_primitive_file;
        double nominal_vel; // m/s
    } MotionPrimitiveParams;

    typedef struct {
        double robot_radius;  // assuming a circular robot
        double fatal_collision_distance;
        // TODO : take in a general footprint
    } RobotDescriptionParams;

    typedef struct {
        int num_leaders;
        std::vector<double> relative_positions;
        std::vector<int> leader_ids;
    } SwarmDescriptionParams;

    typedef struct {
        double env_resolution;
        std::string reference_frame;
        Point3D origin; 
        Point3D max_point;
    } OccupancyGridParams;

    typedef struct {
        bool expansions;
        bool final_path;
    } VisualizationParams;

    /*! \brief Used to grab all parameters used by the planner and organize them
     * into various structs. 
     *
     * In an effort to keep things compartmentalized, each module has its own
     * struct defining what parameters it needs. A lot of the modules use the
     * same parameters, so there's going to be some duplicates. If you're
     * changing these structs, don't try and merge them all together - it's
     * easier to be very clear about what parameters each module takes.
     *
     * The highest level function is the fetch() function, which just calls all
     * parameter getter functions. This catalog is built when the Environment
     * object is first created. 
     */
    class ParameterCatalog {
        public:
            ParameterCatalog();
            void fetch(ros::NodeHandle nh);

            void setMotionPrimitiveParams(MotionPrimitiveParams& mprim);
            void setOccupancyGridParams(OccupancyGridParams& params);
            void setVisualizationParams(VisualizationParams& params);
            void setRobotDescriptionParams(RobotDescriptionParams& params);
            void setSwarmDescriptionParams(SwarmDescriptionParams& params);

            MotionPrimitiveParams m_motion_primitive_params;
            OccupancyGridParams m_occupancy_grid_params;
            VisualizationParams m_visualization_params;
            RobotDescriptionParams m_robot_description_params;
            SwarmDescriptionParams m_swarm_description_params;

        private:
            ros::NodeHandle m_nodehandle;
            bool setFileNameFromParamServer(const std::string param_name, 
                                            std::string* parameter);
        
            void getNextLine(std::ifstream& file, std::stringstream& ss, 
                             std::string& line);
    };
}
