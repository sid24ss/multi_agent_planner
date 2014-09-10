#include <multi_agent_planner/MotionPrimitives/NavMotionPrimitive.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

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
                            GraphStatePtr& successor)
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
    return true;
}

void NavMotionPrimitive::computeTData(const GraphState& source_state,
                                        int leader_id,
                                        GraphStatePtr& successor,
                                        TransitionData& t_data)
{
    // this function fills the intermediate swarm states
    auto start_swarm = source_state.swarm_state();
    auto end_swarm = successor->swarm_state();

    std::vector<SwarmState> interm_swarm_steps;
    // gives only the intermediate swarm steps. And TData should have only those
    // anyway.
    // NOTE: If TData should have the start and the end as well, this is where
    // you need to push them in.
    SwarmState::interpolate(start_swarm, end_swarm, interm_swarm_steps);

    // auto start_leader = start_swarm.robots_pose()[leader_id].getDiscRobotState();

    // for (int i = 0; i < num_interp_steps; ++i) {
    //     // set the leader's interp steps to that of the mprim. (probably not
    //     // even required)
    //     auto robots_list = interm_swarm_steps[i].robots_pose();
    //     DiscRobotState d_leader = robots_list[leader_id].getDiscRobotState();
    //     d_leader.x(start_leader.x() + m_interm_steps[i][RobotStateElement::X]);
    //     d_leader.y(start_leader.y() + m_interm_steps[i][RobotStateElement::Y]);
    //     robots_list[leader_id] = RobotState(d_leader);
    //     interm_swarm_steps[i] = SwarmState(robots_list);
    //     interm_swarm_steps[i].setLeader(start_swarm.getLeader());
    // }
    for (auto& interm_swarm_step : interm_swarm_steps)
        interm_swarm_step.setLeader(start_swarm.getLeader());
    t_data.interm_swarm_steps(interm_swarm_steps);
}
