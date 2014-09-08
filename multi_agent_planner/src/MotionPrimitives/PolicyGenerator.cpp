#include <multi_agent_planner/MotionPrimitives/PolicyGenerator.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

using namespace multi_agent_planner;

PolicyGenerator::PolicyGenerator(CSpaceMgrPtr cspaceptr,
    const RobotDescriptionParams& params)
    :   m_robot_radius (params.robot_radius),
        m_fatal_collision_distance (params.fatal_collision_distance),
        m_collision_space(cspaceptr) {}

bool PolicyGenerator::applyPolicy(const GraphState& leader_moved_state,
                                        int leader_id,
                                        GraphStatePtr& successor)
{
    // reset successor to match the leader_moved_state
    successor.reset(new GraphState(leader_moved_state));

    // go through each robot and apply the policy
    auto robots_list = successor->swarm_state().robots_pose();
    double leader_x = robots_list[leader_id].getContRobotState().x();
    double leader_y = robots_list[leader_id].getContRobotState().y();
    for (size_t i = 0; i < robots_list.size(); ++i) {
        // skip the leader
        if (static_cast<int>(i) == leader_id)
            continue;
        ContRobotState cont_robot_state = robots_list[i].getContRobotState();
        // compute the policy
        double current_dx = leader_x - cont_robot_state.x();
        double current_dy = leader_y - cont_robot_state.y();
        // do this minus the relative position from leader to this guy
        double move_dx = current_dx - SwarmState::REL_POSITIONS[leader_id][i].x();
        double move_dy = current_dy - SwarmState::REL_POSITIONS[leader_id][i].y();
        // now, calculate repulsion from nearby obstacles (or other robots)
        // other robots first
        for (size_t j =0; j < robots_list.size(); j++) {
            if (static_cast<int>(j) == leader_id || j == i)
                continue;
            double dist_to_bot = ContRobotState::distance(cont_robot_state,
                robots_list[j].getContRobotState());
            if (dist_to_bot < 2*m_robot_radius + m_fatal_collision_distance) {
                // we need to repel. How?
                
            }
        }
        // TODO:get the distance to the nearest obstacle. If it is less than some
        // value, repel from that
        cont_robot_state.x(cont_robot_state.x() + move_dx);
        cont_robot_state.y(cont_robot_state.y() + move_dy);

        robots_list[i] = RobotState(cont_robot_state);
    }
    SwarmState successor_swarm(robots_list);
    successor->swarm_state(successor_swarm);
    return true;
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
            );
    }
    return policy_cost;
}