#include <multi_agent_planner/MotionPrimitives/PolicyGenerator.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <costmap_2d/cost_values.h>
#include <cassert>
#include <cmath>

using namespace multi_agent_planner;

PolicyGenerator::PolicyGenerator(CSpaceMgrPtr cspaceptr,
    const RobotDescriptionParams& params)
    :   m_robot_params(params),
        m_collision_space(cspaceptr) {}

bool PolicyGenerator::applyPolicy(const GraphState& leader_moved_state,
                                        int leader_id,
                                        GraphStatePtr& successor,
                                        double leader_movement)
{
    // ROS_DEBUG_NAMED(SEARCH_LOG, "leader_movement : %f", leader_movement);
    // reset successor to match the leader_moved_state
    successor.reset(new GraphState(leader_moved_state));

    // go through each robot and apply the policy
    auto robots_list = successor->swarm_state().robots_pose();
    double leader_x = robots_list[leader_id].getContRobotState().x();
    double leader_y = robots_list[leader_id].getContRobotState().y();
    std::vector<ContRobotState> policies(robots_list.size());
    for (size_t i = 0; i < robots_list.size(); ++i) {
        // ROS_DEBUG_NAMED(POLICYGEN_LOG, "Policy for robot %ld with leader %d", i,
            // leader_id);
        // skip the leader
        if (static_cast<int>(i) == leader_id)
            continue;
        ContRobotState cont_robot_state = robots_list[i].getContRobotState();
        // compute the policy
        // Step 1 : we first want the move-action; this is the action that makes
        // us move where the leader is moving toward
        ContMotion cur_disp(ROBOT_DOF,0);
        cur_disp[RobotStateElement::X] = leader_x - cont_robot_state.x();
        cur_disp[RobotStateElement::Y] = leader_y - cont_robot_state.y();
        // do this minus the relative position from leader to this guy
        ContMotion move_component(ROBOT_DOF, 0);
        move_component[RobotStateElement::X] = cur_disp[RobotStateElement::X] - 
                                    SwarmState::REL_POSITIONS[leader_id][i].x();
        move_component[RobotStateElement::Y] = cur_disp[RobotStateElement::Y] - 
                                    SwarmState::REL_POSITIONS[leader_id][i].y();

        // Step 2: get other robots' influence for this robot
        ContMotion robots_influence(ROBOT_DOF, 0);
        // ContMotion robots_influence = getRobotsInfluence(successor->swarm_state(),
        //                                                 static_cast<int>(i),
        //                                                 leader_id,
        //                                                 leader_movement);
        // ROS_DEBUG_NAMED(POLICYGEN_LOG, "robots influence : %f %f",
        //                             robots_influence[0], robots_influence[1]);
        // Step 3: get the environment's influence
        ContMotion envt_influence = getEnvironmentInfluence(cont_robot_state,
                                                            leader_movement);
        policies[i].x(move_component[RobotStateElement::X] +
                      robots_influence[RobotStateElement::X] + 
                      envt_influence[RobotStateElement::X]);
        policies[i].y(move_component[RobotStateElement::Y] +
                      robots_influence[RobotStateElement::Y] + 
                      envt_influence[RobotStateElement::Y]);
    }
    // now set the robotstate based on the policies
    for(size_t i = 0; i < robots_list.size(); ++i) {
        robots_list[i] = RobotState(robots_list[i].getContRobotState() + policies[i]);
    }
    SwarmState successor_swarm(robots_list);
    successor_swarm.setLeader(leader_id);
    successor->swarm_state(successor_swarm);
    return true;
}

std::vector<double> PolicyGenerator::getRobotsInfluence(
                                                const SwarmState& swarm_state, 
                                                int current_robot_id,
                                                int leader_id,
                                                double leader_movement)
{
    auto robots_list = swarm_state.robots_pose();
    ContMotion robots_influence(ROBOT_DOF, 0);
    auto cont_robot_state = robots_list[current_robot_id].getContRobotState();
    // now, calculate repulsion from nearby obstacles (or other robots)
    // other robots first
    for (size_t j =0; j < robots_list.size(); j++) {
        // skip the leader and the current robot itself
        if (static_cast<int>(j) == leader_id ||
            static_cast<int>(j) == current_robot_id) {
            continue;
        }
        auto repelling_robot = robots_list[j].getContRobotState();
        double dist_to_bot = ContRobotState::distance(cont_robot_state,
            repelling_robot);
        // the criterion here is the distance between robots. Not the center
        // to center distance. Therefore, we look at the
        // neighbor_influence_distance as the distance external to the robots;
         // hence the 2r + neighbor_influence_distance
        double weight_r = 
            (dist_to_bot - 2*m_robot_params.robot_radius
            - m_robot_params.neighbor_influence_distance) / 
            (m_robot_params.fatal_collision_distance - m_robot_params.
            neighbor_influence_distance);
        if (weight_r <= 0)
            continue;
        // we need to repel.
        // compute hinge loss.
        // since we know the magnitude of change in the x and y
        // directions, we only need to find the sign of change.
        double change_mag = leader_movement / SQRT_2;
        // find the directions
        double x_dir = static_cast<double>(sgn<double>(cont_robot_state.x() - repelling_robot.x()));
        double y_dir = static_cast<double>(sgn<double>(cont_robot_state.y() - repelling_robot.y()));
        robots_influence[RobotStateElement::X] += x_dir * weight_r * change_mag;
        robots_influence[RobotStateElement::Y] += y_dir * weight_r * change_mag;
    }
    return robots_influence;
}

std::vector<double> PolicyGenerator::getEnvironmentInfluence(
                                                const ContRobotState& c_state,
                                                double leader_movement)
{
    ContMotion envt_influence(ROBOT_DOF, 0);
    DiscRobotState d_state(c_state);
    // check if we need to find the influence. This is dictated by the costmap
    // value.
    double weight = static_cast<double>(m_grid[d_state.x()][d_state.y()]) /
                    static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
    if (weight > 1e-5) {
        std::pair<int,int> nearest_cell;
        m_gridsearch->getRoot(d_state.x(), d_state.y(),nearest_cell.first,
                                                    nearest_cell.second);
        double change_mag = m_robot_params.envt_compliance_factor *
                            m_robot_params.nominal_vel /
                            SQRT_2;
        double x_dir = static_cast<double>(sgn<double>(d_state.x() - nearest_cell.first));
        double y_dir = static_cast<double>(sgn<double>(d_state.y() - nearest_cell.second));
        if (std::fabs(x_dir) + std::fabs(y_dir) < 2)
            change_mag *= SQRT_2;
        envt_influence[RobotStateElement::X] += x_dir * weight * change_mag;
        envt_influence[RobotStateElement::Y] += y_dir * weight * change_mag;
    }
    return envt_influence;
}


int PolicyGenerator::computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor)
{
    auto begin = graph_state.swarm_state().robots_pose();
    auto end = successor->swarm_state().robots_pose();

    // sanity check
    assert(begin.size() == end.size());

    // go through each of the robots, skip the leader, see how much it has
    // moved, and compute the cost of moving.
    int policy_cost = 0;
    for (size_t i = 0; i < begin.size(); i++) {
        // skip leader because this is captured in the base cost
        if (static_cast<int>(i) == leader_id)
            continue;
        policy_cost += static_cast<int>(
                METER_TO_MM_MULT * ContRobotState::distance(
                    begin[i].getContRobotState(),
                    end[i].getContRobotState())
                + 0.5
            );
    }
    return policy_cost;
}

void PolicyGenerator::update2DHeuristicMaps(const std::vector<unsigned char>& data)
{
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    std::vector<std::pair<int, int> > init_points;

    m_grid = new unsigned char*[dimX + 1];
    for (int i=0; i < dimX + 1; i++){
        m_grid[i] = new unsigned char[dimY + 1];
        for (int j=0; j < dimY + 1; j++){
            m_grid[i][j] = (data[j*(dimX + 1)+i]);
            if (m_grid[i][j] == costmap_2d::LETHAL_OBSTACLE)
                init_points.push_back(std::make_pair(i,j));
        }
    }
    auto last_pt = init_points.back();
    init_points.pop_back();

    m_gridsearch.reset(new SBPL2DGridSearch(dimX+1, dimY + 1,
        m_occupancy_grid->getResolution()));

    ROS_DEBUG_NAMED(POLICYGEN_LOG, "running the 2D gridsearch for PolicyGenerator");
    m_gridsearch->search(m_grid, costmap_2d::LETHAL_OBSTACLE+1,
        last_pt.first, last_pt.second, 0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, init_points);
}