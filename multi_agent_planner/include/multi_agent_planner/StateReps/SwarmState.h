#pragma once
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/ParameterCatalog.h>

namespace multi_agent_planner {
    class SwarmState {
        public:
            SwarmState();
            SwarmState(std::vector<RobotState> robots_pose);
            SwarmState(const SwarmState& other);
            
            // equality of SwarmStates is defined as:
            //          same coordinates for all RobotStates
            bool operator==(const SwarmState& other) const;
            bool operator!=(const SwarmState& other) const;

            static bool isDiscreteEqual(const SwarmState& first,
                                        const SwarmState& second);

            std::vector<RobotState> robots_pose() const { return m_robots_pose; };
            void robots_pose(std::vector<RobotState> robots_state) { m_robots_pose = robots_state; };

            void printToDebug(char* logger) const ;
            void printContToDebug(char* logger) const ;

            std::vector<int> coords() const;
            void coords(std::vector<int> coords);

            void setLeader(int l) { m_leader = l; };
            int getLeader() const { return m_leader; };

            void visualize() const;

            static bool interpolate(const SwarmState& start, const SwarmState& end,
                int num_interp_steps,
                std::vector<SwarmState>& interm_swarm_steps);

            static int NUM_ROBOTS;
            // Note : relative positions [from][to]
            static std::vector<std::vector<ContRobotState> > REL_POSITIONS;
            static std::vector<int> LEADER_IDS;
            static void configureSwarmState(const SwarmDescriptionParams& params);

            static SwarmState transformSwarmToPos(std::vector<double> position,
                                                    int pivot = 0);

        private:
            std::vector<RobotState> m_robots_pose;
            int m_leader;
    };
    typedef boost::shared_ptr<SwarmState> SwarmStatePtr;
};