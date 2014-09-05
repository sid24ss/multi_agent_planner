#include <multi_agent_planner/MotionPrimitives/NavMotionPrimitive.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

#define METER_TO_MM_MULT 1000

using namespace multi_agent_planner;

void NavMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Nav Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tbase cost: %d", getBaseCost());
    printEndCoord();
    printIntermSteps();
}

/*! \brief given the graph state, this applies the base motion primitive while
 * filling in the transitiondata information.  since the base motion primitive
 * list contains motion primitives for every possible base theta, let's only use
 * the one corresponding to our particular angle.
 */
bool NavMotionPrimitive::apply(const GraphState& source_state, 
                            int leader_id,
                            GraphStatePtr& successor,
                            TransitionData& t_data)
{
    successor.reset(new GraphState(source_state));
    // must change the leader's state.
    auto robots_list = successor->swarm_state().robots_pose();
    DiscRobotState leader_state = robots_list[leader_id].getDiscRobotState();
    leader_state.x(leader_state.x() + m_end_coord[RobotStateElement::X]);
    leader_state.y(leader_state.y() + m_end_coord[RobotStateElement::Y]);
    robots_list[leader_id] = RobotState(leader_state);
    // make a new swarm and set the successor's swarm state to the updated one
    SwarmState leader_moved_swarm(robots_list);
    successor->swarm_state(leader_moved_swarm);

    GraphStatePtr policy_applied_state;
    applyPolicy(*successor, leader_id, policy_applied_state, t_data);
    successor = policy_applied_state;

    int policy_cost = computePolicyCost(source_state, leader_id, successor, t_data);
    computeTData(source_state, leader_id, successor, t_data);
    t_data.cost(getBaseCost() + policy_cost);

    return true;
}

bool NavMotionPrimitive::applyPolicy(const GraphState& leader_moved_state,
                                        int leader_id,
                                        GraphStatePtr& successor,
                                        TransitionData& t_data)
{
    // reset successor to match the leader_moved_state
    successor.reset(new GraphState(leader_moved_state));

    // go through each robot and apply the policy
    auto robots_list = successor->swarm_state().robots_pose();
    for (size_t i = 0; i < robots_list.size(); ++i) {
        // skip the leader
        if (static_cast<int>(i) == leader_id)
            continue;
        DiscRobotState disc_robot_state = robots_list[i].getDiscRobotState();
        // compute the policy
        // for now, this is just to apply the same prim to the followers too
        disc_robot_state.x(disc_robot_state.x() + m_end_coord[RobotStateElement::X]);
        disc_robot_state.y(disc_robot_state.y() + m_end_coord[RobotStateElement::Y]);
        robots_list[i] = RobotState(disc_robot_state);
    }
    SwarmState successor_swarm(robots_list);
    successor->swarm_state(successor_swarm);
    return true;
}

int NavMotionPrimitive::computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data)
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

void NavMotionPrimitive::computeTData(const GraphState& source_state,
                                        int leader_id,
                                        GraphStatePtr& successor,
                                        TransitionData& t_data)
{
    // this function fills the intermediate swarm states
    auto start_swarm = source_state.swarm_state();
    auto end_swarm = successor->swarm_state();

    int num_interp_steps = static_cast<int>(getIntermSteps().size());

    std::vector<SwarmState> interm_swarm_steps;
    // gives only the intermediate swarm steps. And TData should have only those
    // anyway.
    // NOTE: If TData should have the start and the end as well, this is where
    // you need to push them in.
    SwarmState::interpolate(start_swarm, end_swarm, num_interp_steps, interm_swarm_steps);

    auto start_leader = start_swarm.robots_pose()[leader_id].getContRobotState();

    for (int i = 0; i < num_interp_steps; ++i) {
        // set the leader's interp steps to that of the mprim. (probably not
        // even required)
        auto robots_list = interm_swarm_steps[i].robots_pose();
        ContRobotState c_leader = robots_list[leader_id].getContRobotState();
        c_leader.x(start_leader.x() + m_interm_steps[i][RobotStateElement::X]);
        c_leader.y(start_leader.y() + m_interm_steps[i][RobotStateElement::Y]);
        robots_list[leader_id] = RobotState(c_leader);
        interm_swarm_steps[i] = SwarmState(robots_list);
    }
    t_data.interm_swarm_steps(interm_swarm_steps);
}
