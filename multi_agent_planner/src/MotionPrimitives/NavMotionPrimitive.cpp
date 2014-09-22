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
    leader_moved_swarm.setLeader(source_state.getLeader());
    successor->swarm_state(leader_moved_swarm);
    return true;
}
