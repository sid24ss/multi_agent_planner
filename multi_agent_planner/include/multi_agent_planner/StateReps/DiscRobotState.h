#pragma once
#include <multi_agent_planner/StateReps/ContRobotState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <vector>

namespace multi_agent_planner {
    // forward declare the ContRobotState
    class ContRobotState;

    class DiscRobotState : public OccupancyGridUser {
        public:
            DiscRobotState() : m_coords(ROBOT_DOF,0) { };
            DiscRobotState(std::vector<int> coords);
            DiscRobotState(ContRobotState cont_r_state);

            bool operator==(const DiscRobotState& other) const ;
            bool operator!=(const DiscRobotState& other) const ;

            int x() const { return m_coords[RobotStateElement::X]; };
            int y() const { return m_coords[RobotStateElement::Y]; };
            void x(int x){ m_coords[RobotStateElement::X] = x; };
            void y(int y){ m_coords[RobotStateElement::Y] = y; };

            std::vector<int> coords() const { return m_coords; };

            // std::vector<int>::const_iterator getCoordBegin(){ return m_coords.begin(); };
            // std::vector<int>::const_iterator getCoordEnd(){ return m_coords.end(); };

            static int convertContDistance(double distance);
            ContRobotState getContRobotState() const;

            void printToDebug(char* logger);

        private:
            std::vector<int> m_coords;
    };
}
