#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Utilities.h>
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
    setSwarmDescriptionParams(m_swarm_description_params);
}

void ParameterCatalog::setMotionPrimitiveParams(MotionPrimitiveParams& params){
    // setFileNameFromParamServer("planner/motion_primitive_file", 
    //         &params.motion_primitive_file);
    m_nodehandle.param("planner/nominalvel_mpersecs", params.nominal_vel, 0.5);
    m_nodehandle.param("planner/change_leader_cost", params.change_leader_cost,
        100);
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
    m_nodehandle.param("robot_description/radius", params.robot_radius, 0.15);
    m_nodehandle.param("robot_description/fatal_collision_distance",
                        params.fatal_collision_distance, params.robot_radius/3);
    m_nodehandle.param("robot_description/fatal_distortion_distance",
                        params.fatal_distortion_distance, 0.5);
    m_nodehandle.param("robot_description/neighbor_influence_distance",
                        params.neighbor_influence_distance, 2*params.robot_radius);
    m_nodehandle.param("robot_description/envt_compliance_factor",
                        params.envt_compliance_factor, 2.0);
    m_nodehandle.param("robot_description/leader_attraction_factor",
                        params.leader_attraction_factor, 1.0);
    m_nodehandle.param("robot_description/neighbor_repel_factor",
                        params.neighbor_repel_factor, 1.0);
    params.nominal_vel = m_motion_primitive_params.nominal_vel;
    ROS_DEBUG_NAMED(CONFIG_LOG, "Setting the robot params");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\t robot_radius : %f", params.robot_radius);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\t fatal fatal_collision_distance : %f", params.fatal_collision_distance);
}

void ParameterCatalog::setSwarmDescriptionParams(SwarmDescriptionParams& params)
{
    std::vector<double> default_rel_positions {0,0,  1.5,0,  1.5,1.5,  0.75,0.75,  0,1.5};
    std::vector<int> default_leader_ids {0, 1, 2, 4};
    m_nodehandle.param("swarm_description/relative_positions", 
                            params.relative_positions,default_rel_positions);
    m_nodehandle.param("swarm_description/num_leaders", params.num_leaders, 4);
    m_nodehandle.param("swarm_description/leader_ids",  params.leader_ids,
                                                        default_leader_ids);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Setting the swarm params");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\trel positions : %s", vectorToString(params.relative_positions).c_str());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tnum_leaders : %d", params.num_leaders);
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tleader_ids : %s", vectorToString(params.leader_ids).c_str());
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
