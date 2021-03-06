#pragma once
#include <multi_agent_planner/StateReps/DiscRobotState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <vector>

namespace multi_agent_planner {
    typedef std::vector<double> ContMotion;

    class DiscRobotState;

    class ContRobotState : public OccupancyGridUser {
        public:
            ContRobotState() : m_coords(ROBOT_DOF, 0) {};
            ContRobotState(std::vector<double> coords);
            ContRobotState(const DiscRobotState& coords);

            bool operator==(const ContRobotState& other) const ;
            bool operator!=(const ContRobotState& other) const ;
            ContRobotState operator+(const ContRobotState& other) const ;

            std::vector<double> coords() const { return m_coords; };

            double x() const { return m_coords[RobotStateElement::X]; };
            double y() const { return m_coords[RobotStateElement::Y]; };
            void x(double x);
            void y(double y);

            DiscRobotState getDiscRobotState() const;
            void printToDebug(char* logger);

            static double getResolution(){ return m_occupancy_grid->getResolution(); };

            // TODO : Interpolate? Is this necessary?

            static double distance(const ContRobotState& start, const ContRobotState& end);

        private:
            std::vector<double> m_coords;
    };
}
