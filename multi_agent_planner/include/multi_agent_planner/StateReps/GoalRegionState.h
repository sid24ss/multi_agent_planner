#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <vector>

namespace multi_agent_planner {
    class GoalRegionState {
        public:
            GoalRegionState() {};
            GoalRegionState(const std::vector<double>& goal_state, double tolerance);
            bool isSatisfiedBy(const GraphStatePtr& graph_state) const;
            void setGoal(std::vector<double> goal_state){ m_goal_state =
                goal_state;};
            std::vector<double> getGoalState() {return m_goal_state; }
            bool withinTol(const GraphStatePtr& graph_state) const;
            void storeAsSolnState(const GraphStatePtr& state){ m_full_goal_state = state; };
            void visualize();
            GraphStatePtr getSolnState() const { return m_full_goal_state; }
        private:
            GraphStatePtr m_full_goal_state;
            std::vector<double> m_goal_state;
            double m_tolerance;
    };
    typedef std::shared_ptr<GoalRegionState> GoalRegionStatePtr;
}
