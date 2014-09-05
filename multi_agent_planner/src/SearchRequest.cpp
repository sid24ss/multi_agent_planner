#include <multi_agent_planner/SearchRequest.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Constants.h>

using namespace multi_agent_planner;

SearchRequest::SearchRequest(SearchRequestParamsPtr params){
    m_params = params;
}

bool SearchRequest::isValid(CSpaceMgrPtr& cspace){
    if (m_params->initial_epsilon < 1 || 
        m_params->final_epsilon < 1 || 
        m_params->decrement_epsilon < 0){
        ROS_ERROR_NAMED(INIT_LOG, "Epsilons in search request were set wrong!");
        return false;
    }
    
    if (!cspace->isValid(m_params->swarm_start)){
        ROS_ERROR_NAMED(INIT_LOG, "Starting pose is invalid.");
        return false;
    }
    return true;
}

GoalStatePtr SearchRequest::createGoalState(){
    return std::make_shared<GoalState>(m_params->swarm_goal,
                                                m_params->tolerance);
}
