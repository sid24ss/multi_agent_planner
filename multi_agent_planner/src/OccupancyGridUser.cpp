#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/LoggerNames.h>

using namespace multi_agent_planner;

void OccupancyGridUser::init(OccupancyGridParams& params){
    m_occupancy_grid = std::make_shared<sbpl_arm_planner::OccupancyGrid>(
                                                          params.max_point.x, 
                                                          params.max_point.y,
                                                          params.max_point.z, 
                                                          params.env_resolution,
                                                          params.origin.x, 
                                                          params.origin.y,
                                                          params.origin.z);
    ROS_DEBUG_NAMED(INIT_LOG, "occupancy grid initialized");
    m_occupancy_grid->setReferenceFrame(params.reference_frame);
}