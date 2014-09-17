#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <multi_agent_planner/Visualizer.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>

using namespace multi_agent_planner;
using namespace boost;

CollisionSpaceMgr::CollisionSpaceMgr(const RobotDescriptionParams& params)
    : m_robot_radius (params.robot_radius),
      m_fatal_collision_distance (params.fatal_collision_distance),
      m_fatal_distortion_distance(params.fatal_distortion_distance)
{ }

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
        if (!isValid(robot_state)) {
            return false;
        }
    }
    if (!checkRobotRobotCollision(swarm_state))
        return false;
    return checkSwarmDistortion(swarm_state);
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
    int z_i = NOMINAL_Z/ContRobotState::getResolution();

    // check if it is within bounds
    if (!m_occupancy_grid->isInBounds(x_i, y_i, z_i))
        return false;

    // check if it is in collision
    // NOTE: assumes that the robot is a simple sphere with radius m_robot_radius
    // (radius was set from the param server in the constructor)
    // TODO : Extent to collision check various footprints
    double dist_temp = m_occupancy_grid->getDistance(x_i, y_i, z_i);
    if (dist_temp <= m_robot_radius)
        return false;

    // no collisions!
    return true;
}

/**
 * @brief checks for internal validity of the swarm state
 * @details checks for internal collisions
 * 
 * @param swarm_state the swarm state you want to collision check
 * @return true if there is NO collision. false if it collides.
 */
bool CollisionSpaceMgr::checkRobotRobotCollision(const SwarmState& swarm_state) const
{
    auto robots_list = swarm_state.robots_pose();
    for (size_t i = 0; i < robots_list.size() - 1; i++){
        for (size_t j = i+1; j < robots_list.size(); j++) {
            double dist = ContRobotState::distance(
                                    robots_list[i].getContRobotState(),
                                    robots_list[j].getContRobotState());
            if (dist <= 2*m_robot_radius + m_fatal_collision_distance)
                return false;
        }
    }
    return true;
}

/**
 * @brief checks if the swarm hasn't distorted too much
 * @details invalid if any robot has moved too much from the relative position
 * it has to be in
 * 
 * @param swarm_state the input swarm pose
 * @return false if it has distorted too much, true if not
 */
bool CollisionSpaceMgr::checkSwarmDistortion(const SwarmState& swarm_state) const
{
    auto robots_list = swarm_state.robots_pose();
    for (size_t i = 0; i < static_cast<size_t>(SwarmState::NUM_ROBOTS); i++) {
        for (size_t j = 0; j < static_cast<size_t>(SwarmState::NUM_ROBOTS); j++) {
            // get the relative position it should be in
            auto desired_coords = SwarmState::REL_POSITIONS[i][j].coords();
            double desired_norm = vectorNorm(desired_coords);
            // get the current distance
            double current_norm = ContRobotState::distance(
                                            robots_list[i].getContRobotState(),
                                            robots_list[j].getContRobotState()
                                        );
            if (current_norm > desired_norm + m_fatal_distortion_distance)
                return false;
        }
    }
    return true;
}
