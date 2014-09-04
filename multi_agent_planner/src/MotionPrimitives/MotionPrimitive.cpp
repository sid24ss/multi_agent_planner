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
