#pragma once
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/MotionPrimitives/NavMotionPrimitive.h>
#include <multi_agent_planner/MotionPrimitives/NavAdaptiveMotionPrimitive.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <vector>

namespace multi_agent_planner {
    typedef std::vector<MotionPrimitivePtr> MPrimList;
    class MotionPrimitivesMgr {
        public:
            MotionPrimitivesMgr() {};
            MotionPrimitivesMgr(GoalStatePtr& goal);
            void setMprimParams(const MotionPrimitiveParams& params);
            bool loadMPrims();
            MPrimList getMotionPrims() { return m_all_mprims; };
        private:
            void loadNavPrims(MPrimList& nav_mprims);

            MPrimList m_all_mprims;

            MotionPrimitiveParams m_params;
            GoalStatePtr m_goal;
    };
}
