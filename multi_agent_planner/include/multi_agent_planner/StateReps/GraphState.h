#pragma once
#include <multi_agent_planner/StateReps/RobotState.h>

namespace multi_agent_planner {
    typedef std::vector<int> GraphStateMotion;
    class GraphState {
        public:
            GraphState(std::vector<RobotState> robots_pose);
            
            // equality of graphstates is defined as:
            //          same coordinates for all robots.
            bool operator==(const GraphState& other) const;
            bool operator!=(const GraphState& other) const;

            int id() const { return m_id; };
            void id(int id) { m_id = id; };

            std::vector<RobotState> robots_pose() const { return m_robots_pose; };
            void robots_pose(std::vector<RobotState> robots_state) { m_robots_pose = robots_state; };

            bool applyMPrim(const GraphStateMotion& mprim);

            void printToDebug(char* logger) const ;
            void printContToDebug(char* logger) const ;

        private:
            int m_id;
            SwarmState m_swarm_state;
            std::vector<int> m_coords;
    };
    typedef boost::shared_ptr<GraphState> GraphStatePtr;
};