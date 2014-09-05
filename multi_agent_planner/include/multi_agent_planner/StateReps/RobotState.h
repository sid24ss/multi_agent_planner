#pragma once
#include <memory>
#include <multi_agent_planner/StateReps/DiscRobotState.h>
#include <multi_agent_planner/StateReps/ContRobotState.h>

namespace multi_agent_planner {
    class RobotState {
        public:
            bool operator==(const RobotState& other) const;
            bool operator!=(const RobotState& other) const;

            RobotState(){};
            RobotState(std::vector<double> values);
            RobotState(ContRobotState cont_state);
            RobotState(DiscRobotState cont_state);

            DiscRobotState getDiscRobotState() const { return m_cont_robot_state; };
            ContRobotState getContRobotState() const { return m_cont_robot_state; };

            void printToDebug(char* log_level) const;
            void printToInfo(char* log_level) const;
            void printToFile(FILE *& path) const;

            void visualize(bool leader) const;

            static std::vector<DiscRobotState> getDiscStates(
                std::vector<RobotState> robot_states);

            static bool interpolate(const RobotState& start, const RobotState& end,
                            int num_interp_steps,
                            std::vector<RobotState>& interm_robot_steps);
            static void vectorToRobotStates(std::vector<double> values,
                                        std::vector<RobotState>& robot_states);

        private:
            ContRobotState m_cont_robot_state;
    };
    typedef std::shared_ptr<RobotState> RobotPosePtr;
}
