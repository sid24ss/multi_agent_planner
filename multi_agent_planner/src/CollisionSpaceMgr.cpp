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

bool CollisionSpaceMgr::loadMap(const vector<Eigen::Vector3d>& points){
    m_occupancy_grid->addPointsToField(points);
    return true;
}

bool CollisionSpaceMgr::isValid(RobotState& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.left_arm().getAngles(&l_arm);
    robot_pose.right_arm().getAngles(&r_arm);
    DiscBaseState discbody_pose = robot_pose.base_state();
    BodyPose body_pose = robot_pose.base_state().getBodyPose();

    double dist_temp;
    int debug_code;
    ROS_DEBUG_NAMED(CSPACE_LOG, "collision checking pose");
    ROS_DEBUG_NAMED(CSPACE_LOG, "body pose is %f %f %f", body_pose.x, 
                                body_pose.y, body_pose.z);
    robot_pose.printToDebug(CSPACE_LOG);
    // Visualizer::pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, 
                                    // std::string("planner"), 0);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, false, dist_temp, 
                                    debug_code);
}

bool CollisionSpaceMgr::isValid(ContBaseState& base, RightContArmState& r_arm, 
                                LeftContArmState& l_arm){
    vector<double> l_arm_v;
    vector<double> r_arm_v;
    l_arm.getAngles(&l_arm_v);
    r_arm.getAngles(&r_arm_v);
    double dist_temp;
    int debug_code;
    BodyPose bp = base.body_pose();
    return m_cspace->checkAllMotion(l_arm_v, r_arm_v, bp, false, dist_temp, 
                                    debug_code);
}

/*! \brief Given the transition data from a state expansion, this does a smart
 * collision check on the successor.
 *
 * If the motion primitive that generated this successor only moves the base,
 * then we don't need to collision check the arms against each other. If only
 * the arm moves, then we don't need to collision check the base for anything.
 *
 * TODO bounds check spine, bounds check base
 */
bool CollisionSpaceMgr::isValidSuccessor(const GraphState& successor,
                                         const TransitionData& t_data){
    RobotState pose = successor.robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = false;
    double dist;
    int debug;

    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                          t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
    if (onlyBaseMotion){
        return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist,
                                         debug);
    } else if (onlyArmMotion){
        bool isvalid = m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, 
                                                 verbose, dist, debug);
        return isvalid;
    } else if (t_data.motion_type() == MPrim_Types::TORSO){
        return m_cspace->checkSpineMotion(l_arm, r_arm, body_pose, verbose, 
                                          dist, debug);
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
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
bool CollisionSpaceMgr::checkCollision(const RobotState& robot_state)
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
}