#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/Visualizer.h>

using namespace multi_agent_planner;

GoalState::GoalState(const SwarmState& goal_state, double tolerance)
    : m_goal_state(goal_state), m_tolerance(tolerance) { }

bool GoalState::withinTol(const GraphStatePtr& graph_state) const {
    int d_tol = static_cast<int>(
        (m_tolerance - 0.005)/ContRobotState::getResolution());

    SwarmState current_state = graph_state->swarm_state();
    auto current_coords = current_state.coords();
    auto goal_state_coords = m_goal_state.coords();
    bool within_tol = true;
    for (size_t i = 0; i < current_coords.size() && within_tol; i++) {
        within_tol = within_tol
            && (std::abs(current_coords[i] - goal_state_coords[i]) < d_tol);
    }
    return within_tol;
}

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state) const {
    return withinTol(graph_state);
}

void GoalState::visualize(){
    m_goal_state.visualize();
}