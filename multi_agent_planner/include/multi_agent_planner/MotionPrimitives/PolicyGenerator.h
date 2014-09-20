#pragma once
#include <multi_agent_planner/MotionPrimitives/MotionPrimitive.h>
#include <multi_agent_planner/Heuristics/2Dgridsearch.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/TransitionData.h>
#include <multi_agent_planner/Constants.h>
#include <cassert>

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
                                GraphStatePtr& successor);

            int computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor);
            void update2DHeuristicMaps(const std::vector<unsigned char>& data);
            bool isLeaderChangeRequired(const GraphState& source_state, const
                GraphState& successor, int leader_id, MotionPrimitivePtr mprim);
        private:
            std::vector <double> getFollowLeaderComponent(
                                    const std::vector<RobotState>& robots_list,
                                    int robot_id,
                                    int leader_id);
            std::vector <double> getLeaderComponent(
                                    const std::vector<RobotState>& robots_list,
                                    int robot_id,
                                    int leader_id);
            std::vector<double> getRobotsInfluence(
                                    const std::vector<RobotState>& robots_list, 
                                    int current_robot_id,
                                    int leader_id);
            std::vector<double> getEnvironmentInfluence(const ContRobotState& c_state);
            ContRobotState getRobotPolicy(const std::vector<RobotState>& robots_list, int leader_id, int robot_id);
            std::unique_ptr<SBPL2DGridSearch> m_gridsearch;
            unsigned char** m_grid;
            RobotDescriptionParams m_robot_params;
            double m_robot_radius;
            double m_fatal_collision_distance;
            CSpaceMgrPtr m_collision_space;
    };
    typedef std::shared_ptr<PolicyGenerator> PolicyGeneratorPtr;
}
