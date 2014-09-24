#pragma once
// #include <sbpl/headers.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/Heuristics/AbstractHeuristic.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <memory>

namespace multi_agent_planner {
    class EuclideanHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser{
        public:
            EuclideanHeuristic();
            ~EuclideanHeuristic();
            // TODO : modify setGoal
            void setGoal(GoalState& state);
            void setGoal(int x, int y);
            void loadMap(const std::vector<unsigned char>& data);
            int getGoalHeuristic(GraphStatePtr state, int leader_id);
            void update2DHeuristicMap(const std::vector<unsigned char>& data);
        protected:
            std::vector<int> m_goal;
    };
    typedef std::shared_ptr<EuclideanHeuristic> EuclideanHeuristicPtr;
}
