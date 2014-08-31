#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/Constants.h>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

GraphState::GraphState(std::vector<RobotState> robots_pose) : m_robot_pose(robots_pose){ }

bool GraphState::operator==(const GraphState& other) const{
    return m_coords == other.coords
}

bool GraphState::operator!=(const GraphState& other) const {
    return !(*this == other);
}

/*! \brief applies a generic mprim vector to this graph state.
 */
bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    // the mprim is a vector of size ROBOT_DOF * NUM_ROBOTS
    return true;
}

void GraphState::printToDebug(char* logger) const {
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    DiscObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();


    ROS_DEBUG_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());

    ROS_DEBUG_NAMED(logger, "\t%d %d %d %d %d %d %d %d %d %d %d %d",
                    obj_state.x(),
                    obj_state.y(),
                    obj_state.z(),
                    obj_state.roll(),
                    obj_state.pitch(),
                    obj_state.yaw(),
                    m_robot_pose.right_free_angle(),
                    m_robot_pose.left_free_angle(),
                    m_robot_pose.base_state().x(),
                    m_robot_pose.base_state().y(),
                    m_robot_pose.base_state().z(),
                    m_robot_pose.base_state().theta());
}

void GraphState::printContToDebug(char* logger) const {
    ContObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    ContObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();
    ContBaseState base_state = m_robot_pose.base_state();
    ROS_DEBUG_NAMED(logger, "object in map %f %f %f %f %f %f",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());
                    
    ROS_DEBUG_NAMED(logger, "\t%f %f %f %f %f %f %f %f %f %f %f %f",
                    obj_state.x(),
                    obj_state.y(),
                    obj_state.z(),
                    obj_state.roll(),
                    obj_state.pitch(),
                    obj_state.yaw(),
                    m_robot_pose.right_arm().getUpperArmRollAngle(),
                    m_robot_pose.left_arm().getUpperArmRollAngle(),
                    base_state.x(),
                    base_state.y(),
                    base_state.z(),
                    base_state.theta());
}

DiscObjectState GraphState::getObjectStateRelMap() const {
    return m_robot_pose.getObjectStateRelMap();
}

DiscObjectState GraphState::getObjectStateRelBody() const {
    return m_robot_pose.getObjectStateRelBody();
}

