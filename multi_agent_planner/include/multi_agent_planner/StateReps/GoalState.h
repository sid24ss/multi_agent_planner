#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <vector>

namespace multi_agent_planner {
    class GoalState {
        public:
            GoalState() {};
            GoalState(const SwarmState& goal_state, double tolerance);
            bool isSatisfiedBy(const GraphStatePtr& graph_state) const;
            SwarmState getSwarmState() const { return m_goal_state; };
            void setGoal(SwarmState goal_state){ m_goal_state =
                goal_state;};
            bool withinTol(const GraphStatePtr& graph_state) const;
            void visualize();
        private:
            // Not underspecified at the moment.
            SwarmState m_goal_state;
            double m_tolerance;
    };
    typedef std::shared_ptr<GoalState> GoalStatePtr;
}
