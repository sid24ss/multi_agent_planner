#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/LoggerNames.h>
#include <boost/filesystem.hpp>
#include <log4cxx/logger.h>

using namespace boost::filesystem;
using namespace multi_agent_planner;
using namespace std;


ParameterCatalog::ParameterCatalog() {
}

void ParameterCatalog::fetch(ros::NodeHandle nh){
    ROS_INFO_NAMED(CONFIG_LOG, "fetching parameters from namespace %s", 
                                nh.getNamespace().c_str());
    m_nodehandle = nh;
    // TODO clean this up, setmotionprimitive needs to be run before parse
    // stuff!
    setOccupancyGridParams(m_occupancy_grid_params);
    setMotionPrimitiveParams(m_motion_primitive_params);
    setVisualizationParams(m_visualization_params);
    setRobotDescriptionParams(m_robot_description_params);
    
}

void ParameterCatalog::setMotionPrimitiveParams(MotionPrimitiveParams& params){
    setFileNameFromParamServer("planner/motion_primitive_file", 
            &params.motion_primitive_file);
    m_nodehandle.param("planner/nominalvel_mpersecs", params.nominal_vel, 0.5);
    params.env_resolution = m_occupancy_grid_params.env_resolution;
}

void ParameterCatalog::setOccupancyGridParams(OccupancyGridParams& params){
    m_nodehandle.param("collision_space/resolution", params.env_resolution, 0.02);
    m_nodehandle.param("collision_space/reference_frame", params.reference_frame, 
                            std::string("base_link"));
    m_nodehandle.param("collision_space/occupancy_grid/origin_x", params.origin.x,-0.6);
    m_nodehandle.param("collision_space/occupancy_grid/origin_y", params.origin.y,-1.15);
    m_nodehandle.param("collision_space/occupancy_grid/origin_z", params.origin.z,-0.05);
    m_nodehandle.param("collision_space/occupancy_grid/size_x", params.max_point.x,1.6);
    m_nodehandle.param("collision_space/occupancy_grid/size_y", params.max_point.y,1.8);
    m_nodehandle.param("collision_space/occupancy_grid/size_z", params.max_point.z,1.4);
    
    ROS_DEBUG_NAMED(CONFIG_LOG, "Occupancy grid parameter");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tsize: %f %f %f ",
                          params.max_point.x, 
                          params.max_point.y,
                          params.max_point.z);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tresolution: %f ",
                          params.env_resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\torigin: %f %f %f",  
                          params.origin.x, 
                          params.origin.y,
                          params.origin.z);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\treference frame: %s", 
                    params.reference_frame.c_str());
}

void ParameterCatalog::setVisualizationParams(VisualizationParams& params){
    m_nodehandle.param("visualizations/expansions", params.expansions, false);
    m_nodehandle.param("visualizations/final_path", params.final_path, false);
    
    ROS_DEBUG_NAMED(CONFIG_LOG, "Setting VisualizationParams");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\texpansions: %d", params.expansions);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tfinal_path: %d", params.final_path);
}

void ParameterCatalog::setRobotDescriptionParams(RobotDescriptionParams& params)
{
    m_nodehandle.param("robot_description/radius", params.robot_radius, 0.3);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Setting the robot params");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\t robot_radius : %f", params.robot_radius);
}

void ParameterCatalog::getNextLine(ifstream& file, stringstream& ss, 
                                   string& line)
{
    getline(file, line);
    ss.str(line);
    ss.clear();
}

bool ParameterCatalog::setFileNameFromParamServer(const std::string param_name, 
                                                  std::string* parameter){
    std::string filename;
    m_nodehandle.param<std::string>(param_name, filename, "");
    path input_path(filename.c_str());
    if (exists(input_path)){
       *parameter = filename;
        ROS_DEBUG_NAMED(CONFIG_LOG, "Pulling in data from %s", filename.c_str());
    } else {
       *parameter = filename;
        ROS_ERROR_NAMED(CONFIG_LOG, "Failed to find file '%s' to load in parameters for %s", 
                        filename.c_str(), param_name.c_str());
        return false;
    }
    return true;
}
