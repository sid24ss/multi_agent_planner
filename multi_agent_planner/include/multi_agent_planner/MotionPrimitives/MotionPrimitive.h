#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/TransitionData.h>
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/Constants.h>
#include <assert.h>

namespace multi_agent_planner {
    /*! \brief Base class for motion primitives. Motion primitives contain user
     * defined motions for generating successors in a search. 
     */
    class MotionPrimitive {
        public:
            MotionPrimitive();
            void setID(int id) { m_id = id; };
            int getID() const { return m_id; };
            // both of these are just for the robot state.
            virtual void setIntermSteps(IntermSteps& coord) { m_interm_steps = coord; };
            virtual IntermSteps getIntermSteps(){ return m_interm_steps; };

            virtual bool apply( const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor) = 0;

            virtual void computeTData( const GraphState& graph_state, 
                                int leader_id,
                                GraphStatePtr& successor,
                                TransitionData& t_data) = 0;

            virtual void print() const = 0;
            virtual void setBaseCost(int cost) { m_base_cost = cost; }
            virtual int getBaseCost() const { return m_base_cost; }

            virtual void printIntermSteps() const;
            virtual void printEndCoord() const;

            virtual void setEndCoord(GraphStateMotion& coord); 
            GraphStateMotion getEndCoord() const { return m_end_coord; };

            // returns the continuous displacement that this mprim causes.
            double getDisplacement();

            // get the type of the primitive
            virtual MPrim_Type::Type getPrimitiveType() const = 0;
        protected:
            int m_id;
            // base cost for moving just the leader robot
            int m_base_cost;
            GraphStateMotion m_end_coord;
            IntermSteps m_interm_steps;
    };
    typedef std::shared_ptr<MotionPrimitive> MotionPrimitivePtr;
}
