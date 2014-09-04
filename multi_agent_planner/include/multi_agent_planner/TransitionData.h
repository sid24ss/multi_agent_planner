#pragma once
#include <multi_agent_planner/StateReps/SwarmState.h>

namespace multi_agent_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    /*! \brief Contains information generated during a state expansion.
     *
     * When a state is expanded, it usually has unique information about the
     * expansion, namely the intermediate states associated with the particular
     * motion primitive. This class wraps it all up so it can be easily used for
     * collision checking and path reconstruction by other objects.
     */
    class TransitionData {
        public:
            TransitionData(){};
            // void successor_id(int id){ m_successor_id = id; };
            // int successor_id(){ return m_successor_id; };

            void interm_swarm_steps(std::vector<SwarmState> steps){ m_swarm_interm_steps = steps; };
            std::vector<SwarmState> interm_swarm_steps() const { return m_swarm_interm_steps; };

            void cost(int cost){ m_cost = cost; };
            int cost() const { return m_cost; };
        private:
            // int m_successor_id;
            int m_cost;

            std::vector<SwarmState> m_swarm_interm_steps;
    };
}
