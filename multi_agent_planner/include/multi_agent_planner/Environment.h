#pragma once
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <multi_agent_planner/HashManager.h>
#include <multi_agent_planner/SearchRequest.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <multi_agent_planner/MotionPrimitives/PolicyGenerator.h>
#include <multi_agent_planner/Heuristics/HeuristicMgr.h>
#include <multi_agent_planner/PathPostProcessor.h>
#include <sbpl/headers.h>
#include <ros/ros.h>
#include <stdexcept>
#include <vector>
#include <memory>

#define EPS1 15
#define EPS2 1.1

namespace multi_agent_planner {
    /*! \brief Implements a complete environment used by the SBPL planner.
     * Contains everything from managing state IDs to collision space
     * information.
     */
    typedef std::pair<int, int> Edge;
    class Environment : public EnvironmentMHA {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            HeuristicMgrPtr getHeuristicMgr(){ return m_heur_mgr; };
            bool configureRequest(SearchRequestParamsPtr search_request_params,
                                  int& start_id, int& goal_id);
            void GetSuccs(int sourceStateID, std::vector<int>* succIDs, 
                std::vector<int>* costs);
            void GetSuccs(int sourceStateID, std::vector<int>* succIDs, 
                std::vector<int>* costs, int leader_id);
            virtual void GetLazySuccs(int sourceStateID, std::vector<int>* succIDs, 
                std::vector<int>* costs, std::vector<bool>* isTrueCost);
            virtual void GetLazySuccs(int sourceStateID, std::vector<int>* succIDs, 
                std::vector<int>* costs, std::vector<bool>* isTrueCost, int leader_id);
            virtual int GetTrueCost(int parentID, int childID);
            std::vector<SwarmState> reconstructPath(std::vector<int> 
                state_ids);
            inline int getLeaderChangeCount() const {return
                m_num_leader_changes; }
            void reset();
            // void setPlannerType(int planner_type);
            PolicyGeneratorPtr getPolicyGenerator() {return m_policy_generator;}

        protected:
            bool setStartGoal(SearchRequestPtr search_request, 
                              int& start_id, int& goal_id);
            int saveFakeGoalState(const GraphStatePtr& graph_state);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);
            void generateStartState(SearchRequestPtr search_request);

            ParameterCatalog m_param_catalog;
            CSpaceMgrPtr m_cspace_mgr;
            HashManagerPtr m_hash_mgr;
            ros::NodeHandle m_nodehandle;
            GoalStatePtr m_goal;
            MotionPrimitivesMgr m_mprims;
            PolicyGeneratorPtr m_policy_generator;
            HeuristicMgrPtr m_heur_mgr;
            std::vector<int> m_leader_ids;
            int m_start_state_id;
            int m_num_leader_changes;

            // int m_planner_type;
            // std::unordered_map<int, PlanningModes::modes> m_action_partition;

        // SBPL interface stuff
        public:
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg){ return true; };
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ throw std::runtime_error("unimplemented");  };
            int  GetGoalHeuristic(int stateID);
            int  GetGoalHeuristic(int leader_id, int stateID);
            int  GetStartHeuristic(int stateID) { throw std::runtime_error("unimplemented"); };
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv(){ return m_hash_mgr->size(); };
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL){};
            void PrintEnv_Config(FILE* fOut){};
            // we'll save the primitive that was used to generate the successor
            // and use the same interpolation later.
            std::map<Edge, MotionPrimitivePtr> m_edges;
    };
}
