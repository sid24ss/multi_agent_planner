#pragma once
#include <multi_agent_planner/Heuristics/2Dgridsearch.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/TransitionData.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

namespace multi_agent_planner {
    /*! \brief Base class for motion primitives. Motion primitives contain user
     * defined motions for generating successors in a search. 
     */
    class PolicyGenerator : public OccupancyGridUser {
        public:
            PolicyGenerator(CSpaceMgrPtr cspaceptr, const RobotDescriptionParams&
                params);
            bool applyPolicy( const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor, double leader_movement);

            int computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor);
            void update2DHeuristicMaps(const std::vector<unsigned char>& data);
        private:
            std::vector<double> getRobotsInfluence(const SwarmState& swarm_state, 
                                        int current_robot_id,
                                        int leader_id,
                                        double leader_movement);
            std::vector<double> getEnvironmentInfluence(const ContRobotState& c_state,
                                            double leader_movement);
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned char** m_grid;
            RobotDescriptionParams m_robot_params;
            double m_robot_radius;
            double m_fatal_collision_distance;
            CSpaceMgrPtr m_collision_space;
    };
    typedef std::shared_ptr<PolicyGenerator> PolicyGeneratorPtr;
}
