#pragma once
#include <multi_agent_planner/MotionPrimitives/MotionPrimitive.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/LoggerNames.h>

namespace multi_agent_planner {
    // The base motion primitives read in from file are different than normal
    // mprims. Instead of having 8 generic motion primitives that are applied at
    // each angle, we have 16*8 total mprims - 8 motion primitives for each 16
    // thetas. This is done because applying a generic motion to any theta may
    // not result in the same absolute movement due to discretization error.
    //
    class NavMotionPrimitive : public MotionPrimitive { 
        public:
            virtual bool apply(const GraphState& graph_state,
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data);
            virtual bool applyPolicy(const GraphState& graph_state,
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data);
            virtual void print() const ;
            virtual int computePolicyCost(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data);
            virtual void computeTData(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data);
    };
    typedef std::shared_ptr<NavMotionPrimitive> NavMotionPrimitivePtr;
}
