#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <stdexcept>
#include <memory>
#include <vector>

namespace multi_agent_planner {
    /*! \brief The Abstract Heuristic class that has to be inherited
     * for use in the environment.
     */
    class AbstractHeuristic{
        public:
            AbstractHeuristic() : m_cost_multiplier(1) {};

            // The function that has to return the heuristic value at the queried graph state
            virtual int getGoalHeuristic(GraphStatePtr state, int leader_id) = 0;
            virtual void setGoal(GoalState& state) = 0;
            virtual void setGoal(int x, int y) = 0;

            // For 3D heuristics that need the obstacles - this function has to be implemented for the heuristic to receive the call for obstacle grid update.
            virtual void update3DHeuristicMap() {};

            // For 2D heuristics at the base that need only the map - this function has to be implemented for the heuristic to receive the call for map update.
            virtual void update2DHeuristicMap(const std::vector<unsigned char>& data) {};

            // Set the cost_multiplier
            virtual inline void setCostMultiplier(const int cost_multiplier) { m_cost_multiplier
                = cost_multiplier; };

            // Get the cost multiplier
            virtual inline int getCostMultiplier(){ return m_cost_multiplier; };

        private:
            int m_cost_multiplier;
    };
    typedef std::shared_ptr<AbstractHeuristic> AbstractHeuristicPtr;
}
