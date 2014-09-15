#pragma once
#include <multi_agent_planner/MotionPrimitives/MotionPrimitive.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/LoggerNames.h>
#include <stdexcept>

namespace multi_agent_planner {

    class ChangeLeaderPrimitive : public MotionPrimitive { 
        public:
            virtual bool apply(const GraphState& graph_state,
                                int leader_id,
                                GraphStatePtr& successor)
            { throw std::runtime_error("unimplemented"); }
            virtual void print() const {
                throw std::runtime_error("unimplemented"); }
            virtual void computeTData(const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data)
            { }
            virtual MPrim_Type::Type getPrimitiveType() const { return
                MPrim_Type::CHANGE_LEADER; };
    };
    typedef std::shared_ptr<ChangeLeaderPrimitive> ChangeLeaderPrimitivePtr;
}
