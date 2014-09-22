#include <multi_agent_planner/MotionPrimitives/MotionPrimitive.h>
#include <multi_agent_planner/LoggerNames.h>
#include <boost/foreach.hpp>
#include <sstream>

using namespace multi_agent_planner;

MotionPrimitive::MotionPrimitive() : m_end_coord(ROBOT_DOF,0) {
}

void MotionPrimitive::setEndCoord(GraphStateMotion& coord) { 
    assert((int)coord.size()==ROBOT_DOF); 
    m_end_coord = coord;
}

double MotionPrimitive::getDisplacement() {
    ContRobotState c_origin;
    ContRobotState c_displaced(m_end_coord);
    return ContRobotState::distance(c_origin, c_displaced);
}

void MotionPrimitive::computeTData(const GraphState& source_state,
                                        int leader_id,
                                        GraphStatePtr& successor,
                                        TransitionData& t_data)
{
    // this function fills the intermediate swarm states
    auto start_swarm = source_state.swarm_state();
    auto end_swarm = successor->swarm_state();

    std::vector<SwarmState> interm_swarm_steps;
    // gives only the intermediate swarm steps. And TData should have only those
    // anyway.
    // NOTE: If TData should have the start and the end as well, this is where
    // you need to push them in.
    SwarmState::interpolate(start_swarm, end_swarm, interm_swarm_steps);

    for (auto& interm_swarm_step : interm_swarm_steps)
        interm_swarm_step.setLeader(start_swarm.getLeader());
    t_data.interm_swarm_steps(interm_swarm_steps);
}

void MotionPrimitive::printIntermSteps() const {
    BOOST_FOREACH(auto step, m_interm_steps){
        std::stringstream ss;
        ss << "\t";
        BOOST_FOREACH(auto coord, step){
            ss << coord << " ";
        }
        ROS_DEBUG_NAMED(MPRIM_LOG, "\tinterm steps %s", ss.str().c_str());
    }
}

void MotionPrimitive::printEndCoord() const {
    std::stringstream ss;
    ss << "\t";
    BOOST_FOREACH(auto coord, m_end_coord){
        ss << coord << " ";
    }
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tend coord %s", ss.str().c_str());
}
