#pragma once
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
                                GraphStatePtr& successor);

            int computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor);
        protected:
            double m_robot_radius;
            double m_fatal_collision_distance;
            CSpaceMgrPtr m_collision_space;
    };
    typedef std::shared_ptr<PolicyGenerator> PolicyGeneratorPtr;
}
