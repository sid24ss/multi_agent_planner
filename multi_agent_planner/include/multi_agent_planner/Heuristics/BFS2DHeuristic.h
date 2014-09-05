#pragma once
// #include <sbpl/headers.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/Heuristics/AbstractHeuristic.h>
#include <multi_agent_planner/Heuristics/2Dgridsearch.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <memory>

namespace multi_agent_planner {
    class BFS2DHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser{
        public:
            BFS2DHeuristic();
            ~BFS2DHeuristic();
            // TODO : modify setGoal
            void setGoal(GoalState& state);
            void setGoal(int x, int y);
            void loadMap(const std::vector<unsigned char>& data);
            int getGoalHeuristic(GraphStatePtr state, int leader_id);
            void update2DHeuristicMap(const std::vector<unsigned char>& data);

            // static void visualizeCenter(int x, int y);
            void setUniformCostSearch(bool ucs = false) {
                m_gridsearch->setUniformCostSearch(ucs); }

        protected:
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned int m_size_col;
            unsigned int m_size_row;
            unsigned char** m_grid;
    };
    typedef std::shared_ptr<BFS2DHeuristic> BFS2DHeuristicPtr;
}
