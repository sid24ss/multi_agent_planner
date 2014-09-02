#pragma once
#include <multi_agent_planner/StateReps/SwarmState.h>

namespace multi_agent_planner {
    typedef std::vector<int> GraphStateMotion;
    class GraphState {
        public:
            GraphState(SwarmState swarm_state);
            // GraphState(std::vector<int/double> coords) TODO
            
            // equality of graphstates is defined as:
            //          same coordinates for all robots.
            bool operator==(const GraphState& other) const;
            bool operator!=(const GraphState& other) const;

            int id() const { return m_id; };
            void id(int id) { m_id = id; };

            SwarmState swarm_state() const { return m_swarm_state; };
            void swarm_state(SwarmState swarm_state) { m_swarm_state = swarm_state; };

            bool applyMPrim(const GraphStateMotion& mprim);

            void printToDebug(char* logger) const ;
            // void printContToDebug(char* logger) const ;

            void updateStateFromSwarmState();
            void updateSwarmStateFromGraphState();

            std::vector<int> getCoords() const { return m_coords; };

        private:
            int m_id;
            SwarmState m_swarm_state;
            std::vector<int> m_coords;
    };
    typedef std::shared_ptr<GraphState> GraphStatePtr;
};