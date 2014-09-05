#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Visualizer.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

using namespace multi_agent_planner;
using namespace boost;

CollisionSpaceMgr::CollisionSpaceMgr(const RobotDescriptionParams& params)
    : m_robot_radius (params.robot_radius) { }

/*! \brief Updates the internal collision map of the collision checker.
 */
void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i < (int)map.boxes.size(); i++){
        Eigen::Vector3d vect;
        vect << map.boxes[i].center.x,
        map.boxes[i].center.y,
        map.boxes[i].center.z;
        points.push_back(vect);
    }
    m_occupancy_grid->addPointsToField(points);
}

bool CollisionSpaceMgr::loadMap(const std::vector<Eigen::Vector3d>& points){
    m_occupancy_grid->addPointsToField(points);
    return true;
}

bool CollisionSpaceMgr::isValid(const RobotState& robot_pose) const {
    return checkCollision(robot_pose);
}

bool CollisionSpaceMgr::isValid(const SwarmState& swarm_state) const {
    for (auto& robot_state : swarm_state.robots_pose()) {
        if (!isValid(robot_state))
            return false;
    }
    return true;
}

/**
 * @brief checks collision on the successor state
 */

// TODO : Smart TransitionData where you keep a track of which robots moved, so
// that you don't have to collision check the whole swarm.
bool CollisionSpaceMgr::isValidSuccessor(const GraphState& successor) const {
    return isValid(successor.swarm_state());
}

bool CollisionSpaceMgr::isValidTransitionStates(const TransitionData& t_data) const
{
    for (auto& swarm_state : t_data.interm_swarm_steps()) {
        if (!isValid(swarm_state))
            return false;
    }
    return true;
}

/**
 * @brief checks for validity of the robot state
 * @details assumes that the robot is a simple sphere, for now. Checks the
 * validity of the continuous state.
 * 
 * @param robot_state The robot state to check collision for.
 * @return true if there is NO collision. false if it collides.
 */
bool CollisionSpaceMgr::checkCollision(const RobotState& robot_state) const
{
    assert(m_robot_radius > 0);

    // get the coordinates
    DiscRobotState disc_robot_state = robot_state.getDiscRobotState();
    int x_i = disc_robot_state.x();
    int y_i = disc_robot_state.y();
    int z_i = 0;

    // check if it is within bounds
    if (!m_occupancy_grid->isInBounds(x_i, y_i, z_i))
        return false;

    // check if it is in collision
    // NOTE: assumes that the robot is a simple sphere with radius m_robot_radius (
    // this was set from the param server in the constructor)
    // TODO : Extent to collision check various footprints
    if (m_occupancy_grid->getDistance(x_i, y_i, z_i) <= m_robot_radius)
        return false;

    // no collisions!
    return true;
}