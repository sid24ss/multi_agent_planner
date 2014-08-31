#pragma once
#include <memory>
#include <multi_agent_planner/StateReps/DiscRobotState.h>
#include <multi_agent_planner/StateReps/ContRobotState.h>

namespace multi_agent_planner {
    class RobotState {
        public:
            bool operator==(const RobotState& other);
            bool operator!=(const RobotState& other);

            RobotState(){};
            RobotState(ContRobotState cont_state);

            DiscRobotState getDiscRobotState() const { return m_cont_robot_state; };
            ContRobotState getContRobotState() const { return m_cont_robot_state; };

            void printToDebug(char* log_level) const;
            void printToInfo(char* log_level) const;
            void printToFile(FILE *& path) const;

            void visualize(bool leader = false);

            // TODO : Do we need an interpolate function?

        private:
            ContRobotState m_cont_robot_state;
    };
    typedef std::shared_ptr<RobotState> RobotPosePtr;
}
