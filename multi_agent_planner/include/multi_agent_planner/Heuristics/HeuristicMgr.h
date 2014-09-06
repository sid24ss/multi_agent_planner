#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/Heuristics/AbstractHeuristic.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Visualizer.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <sbpl/utils/utils.h>
#include <memory>
#include <vector>
#include <unordered_map>

namespace multi_agent_planner {

    // for the heuristic map
    typedef std::unordered_map <std::string, int> stringintmap;
    /*! \brief The manager class that handles all the heuristics.
     */
    class HeuristicMgr : public OccupancyGridUser {
        public:
            HeuristicMgr();
            ~HeuristicMgr();
            // HeuristicMgr(CSpaceMgrPtr cspace_mgr);

            // The master function that initializes all the heuristics you
            // want.
            void initializeHeuristics();

            // Add methods for all possible kinds of heuristics. Whenever a new
            // heuristic type is added, a corresponding add<type>Heur() method
            // needs to be added here. Returns the id of the heuristic in the
            // internal m_heuristics vector.
            void add2DHeur(std::string name, const int cost_multiplier = 1);

            // Updates the collision map for the heuristics that need them.
            // Doesn't take in an argument because each 3D heuristic shares the
            // occupancy grid singleton.
            void update3DHeuristicMaps();

            // Updates the 2D map for the heuristics that need them
            void update2DHeuristicMaps(const std::vector<unsigned char>& data);

            // TODO: Multiple goals should just take the goal state and the heuristic ID.
            void setGoal(GoalState& state);

            // Get the heuristic value
            void getGoalHeuristic(const GraphStatePtr& state,
                std::unique_ptr<stringintmap>& values);
            void getGoalHeuristic(const GraphStatePtr& state,
                std::vector<int>& values);
            int getGoalHeuristic(const GraphStatePtr& state, std::string name,
                int leader_id);

            // MHA stuff

            void reset();
            // void setPlannerType(int planner_type);

            // prints a complete summary of all the heuristics.
            void printSummaryToInfo(char* logger);
            void printSummaryToDebug(char* logger);

        inline void setCollisionSpaceMgr(CSpaceMgrPtr cspace_mgr){ m_cspace_mgr = cspace_mgr;};
 
        private:
            GoalState m_goal;
            std::vector<AbstractHeuristicPtr> m_heuristics;
            stringintmap m_heuristic_map;

            // MHA stuff
            // int m_planner_type;

            int m_num_leaders;
            std::vector<int> m_leader_ids;
            
            // Saving the goal and the grid for MHA heuristics
            unsigned char** m_grid;
            std::vector<unsigned char> m_grid_data;

            CSpaceMgrPtr m_cspace_mgr;
    };
    typedef std::shared_ptr<HeuristicMgr> HeuristicMgrPtr;
}
