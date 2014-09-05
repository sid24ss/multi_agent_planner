#pragma once
#include <multi_agent_planner/MotionPrimitives/MotionPrimitive.h>
#include <multi_agent_planner/HashManager.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <vector>

namespace multi_agent_planner {
    class PathPostProcessor {
        public:
            PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr);
            std::vector<SwarmState> reconstructPath(
                                            std::vector<int> state_ids,
                                            GoalState& goal_state,
                                            std::map< std::pair<int,int>,
                                            MotionPrimitivePtr>& edge_cache);
            void visualizeFinalPath(std::vector<SwarmState> path);
        private:
            std::vector<SwarmState> getFinalPath(const std::vector<int>& state_ids,
                                            const std::vector<TransitionData>& transition_states,
                                            GoalState& goal_state);
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
    };
}
