#include <multi_agent_planner/Heuristics/EuclideanHeuristic.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <stdexcept>

using namespace multi_agent_planner;

EuclideanHeuristic::EuclideanHeuristic()
    : m_goal(ROBOT_DOF, 0)
{ }

EuclideanHeuristic::~EuclideanHeuristic(){
}

void EuclideanHeuristic::update2DHeuristicMap(const std::vector<unsigned char>& data){
    loadMap(data);
}

void EuclideanHeuristic::loadMap(const std::vector<unsigned char>& data){
}

void EuclideanHeuristic::setGoal(int x, int y) {
    m_goal[RobotStateElement::X] = x;
    m_goal[RobotStateElement::Y] = y;
}

void EuclideanHeuristic::setGoal(GoalState& goal_state){
    throw std::runtime_error("You shouldn't be calling this!");
}

int EuclideanHeuristic::getGoalHeuristic(GraphStatePtr state, int leader_id) {
    // get the leader's coordinates
    DiscRobotState d_robot_state = 
            state->swarm_state().robots_pose()[leader_id].getDiscRobotState();
    int x = d_robot_state.x();
    int y = d_robot_state.y();
    
    int cost = std::sqrt(std::pow(x - m_goal[RobotStateElement::X], 2)
                        + std::pow(y - m_goal[RobotStateElement::Y], 2)) + 0.5;

    return getCostMultiplier()*cost;
}
