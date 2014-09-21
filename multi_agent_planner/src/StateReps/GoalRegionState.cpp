#include <multi_agent_planner/StateReps/GoalRegionState.h>
#include <multi_agent_planner/Visualizer.h>
#include <multi_agent_planner/LoggerNames.h>

using namespace multi_agent_planner;

GoalRegionState::GoalRegionState(const std::vector<double>& goal, double tolerance)
    : m_goal_state(goal), m_tolerance(tolerance) { }

bool GoalRegionState::withinTol(const GraphStatePtr& graph_state) const {
    // ROS_DEBUG_NAMED(SEARCH_LOG, "isGoal : d_tol : %d", d_tol);

    SwarmState current_state = graph_state->swarm_state();
    auto robots_list = current_state.robots_pose();
    // create a temporary cont robot state that serves as the goal state
    ContRobotState goal_state(m_goal_state);
    // ROS_DEBUG_NAMED(SEARCH_LOG, "current state : ");
    // current_state.printToDebug(SEARCH_LOG);
    // ROS_DEBUG_NAMED(SEARCH_LOG, "goal state : ");
    // m_goal_state.printToDebug(SEARCH_LOG);
    bool within_tol = true;
    for (size_t i = 0; i < robots_list.size() && within_tol; i++) {
        auto c_state = robots_list[i].getContRobotState();
        within_tol = within_tol
            && ( ContRobotState::distance(c_state, goal_state) <= m_tolerance);
    }
    return within_tol;
}

bool GoalRegionState::isSatisfiedBy(const GraphStatePtr& graph_state) const {
    return withinTol(graph_state);
}

void GoalRegionState::visualize(){
    Visualizer::swarmVizPtr->visualizeCircle("goal_state",
                                        m_goal_state[RobotStateElement::X],
                                        m_goal_state[RobotStateElement::Y],
                                        m_tolerance,
                                        100);
}