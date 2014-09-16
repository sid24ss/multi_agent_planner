#include <multi_agent_planner/StateReps/ContRobotState.h>
#include <multi_agent_planner/LoggerNames.h>

using namespace multi_agent_planner;
using namespace std;

ContRobotState::ContRobotState(std::vector<double> coords)
{
    x(coords[RobotStateElement::X]);
    y(coords[RobotStateElement::Y]);
};


bool ContRobotState::operator==(const ContRobotState& other) const {
    return (m_coords == other.m_coords);
}

bool ContRobotState::operator!=(const ContRobotState& other) const {
    return !(*this == other);
}

ContRobotState ContRobotState::operator+(const ContRobotState& other) const {
    ContRobotState new_state;
    new_state.x(x() + other.x());
    new_state.y(y() + other.y());
    return new_state;
}

void ContRobotState::x(double x) {
    m_coords[RobotStateElement::X] = std::floor(x*100 + 0.5)/100;
}

void ContRobotState::y(double y) {
    m_coords[RobotStateElement::Y] = std::floor(y*100 + 0.5)/100;
}


ContRobotState::ContRobotState(const DiscRobotState& disc_r_state) :
    m_coords(ROBOT_DOF, 0) {
    double vx, vy, vz;
    m_occupancy_grid->gridToWorld(disc_r_state.x(),
                                  disc_r_state.y(),
                                  0,
                                  vx,
                                  vy,
                                  vz);
    x(vx);
    y(vy);
}

DiscRobotState ContRobotState::getDiscRobotState() const
{
    return DiscRobotState(*this);
}

double ContRobotState::distance(const ContRobotState& start, const ContRobotState& end){
    double dX = end.x() - start.x();
    double dY = end.y() - start.y();
    return pow((pow(dX,2) + pow(dY,2)),.5);
}

void ContRobotState::printToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "%f %f", x(),y());
}