#include <multi_agent_planner/MotionPrimitives/NavAdaptiveMotionPrimitive.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

using namespace multi_agent_planner;

GoalState NavAdaptiveMotionPrimitive::m_goal;

void NavAdaptiveMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Nav Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tbase cost: %d", getBaseCost());
}

/*! \brief given the graph state, this applies the base motion primitive while
 * filling in the transitiondata information.  since the base motion primitive
 * list contains motion primitives for every possible base theta, let's only use
 * the one corresponding to our particular angle.
 */
bool NavAdaptiveMotionPrimitive::apply(const GraphState& source_state, 
                            int leader_id,
                            GraphStatePtr& successor)
{
    if(!nearGoal(source_state))
        return false;
    successor.reset(new GraphState(m_goal.getSwarmState()));
    successor->setLeader(source_state.getLeader());
    return true;
}

bool NavAdaptiveMotionPrimitive::nearGoal(const GraphState& graph_state){
    auto current_coords = graph_state.getCoords();
    GraphState sample_goal(m_goal.getSwarmState());
    auto goal_coords = sample_goal.getCoords();
    int max_dev = 0;
    // skip the last point, because that represents the leader.
    for (size_t i = 0; i < current_coords.size()-1; i++) {
        if (max_dev < std::abs(current_coords[i] - goal_coords[i]))
            max_dev = std::abs(current_coords[i] - goal_coords[i]);
    }
    return (max_dev < 3);
}