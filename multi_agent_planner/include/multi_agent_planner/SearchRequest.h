#pragma once
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <vector>

namespace multi_agent_planner {
    typedef struct {
        double initial_epsilon;
        double final_epsilon;
        double decrement_epsilon;
        double tolerance;
        std::vector<double> swarm_start;
        std::vector<double> swarm_goal;
    } SearchRequestParams;

    enum RequestErrors { 
        VALID_REQUEST,
        INVALID_START,
        INVALID_GOAL,
        INVALID_PARAM
    };

    typedef std::shared_ptr<SearchRequestParams> SearchRequestParamsPtr;
    class SearchRequest {
        public:
            SearchRequest(SearchRequestParamsPtr params);
            bool isValid(CSpaceMgrPtr& cspace);
            SearchRequestParamsPtr m_params;
            GoalStatePtr createGoalState();
    };
    typedef std::shared_ptr<SearchRequest> SearchRequestPtr;
}
