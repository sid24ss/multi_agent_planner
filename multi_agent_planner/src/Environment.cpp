#include <multi_agent_planner/Environment.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/Visualizer.h>
#include <multi_agent_planner/Constants.h>
#include <algorithm>
#include <cassert>

#define GOAL_STATE 1
using namespace multi_agent_planner;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh)
    :   m_hash_mgr(new HashManager(&StateID2IndexMapping)),
        m_nodehandle(nh), m_mprims(m_goal),
        m_heur_mgr(new HeuristicMgr())
{
        m_param_catalog.fetch(nh);
        configurePlanningDomain();
}

/**
 * @brief Resets the environment.
 * @details Intended to be used between calls to subsequent planning
 * requests.
 */
void Environment::reset() {
    m_heur_mgr->reset();
    // m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    m_hash_mgr.reset(new HashManager(&StateID2IndexMapping));
    m_edges.clear();
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id) {
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(
        search_request_params));
    configureQuerySpecificParams(search_request);

    if (!setStartGoal(search_request, start_id, goal_id)) {
        return false;
    }

    return true;
}

int Environment::GetGoalHeuristic(int stateID) {
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(0, stateID);
}

int Environment::GetGoalHeuristic(int q_id, int stateID) {
    // ROS_DEBUG_NAMED(HEUR_LOG, "Queried queue : %d; setting to %d", q_id,
    //     SwarmState::LEADER_IDS.at(q_id - 1));
    // this is the state we want the heuristic for.
    GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);
    // if it is the goal state, return 0.
    if(m_goal->isSatisfiedBy(successor) || stateID == GOAL_STATE){
        return 0;
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "heur asked for: %d, at q_id %d", 
        stateID, q_id);
    successor->printToDebug(HEUR_LOG);
    int heur;
    if (q_id == 0) {
        std::vector<int> values;
        // get the admissible heuristic - max of all the leader ones.
        m_heur_mgr->getGoalHeuristic(successor, values);
        heur = *std::max_element(values.begin(), values.end());
    } else {
        std::stringstream ss;
        ss << "bfs2d_" << SwarmState::LEADER_IDS.at(q_id - 1);
        heur = m_heur_mgr->getGoalHeuristic(successor, ss.str(), SwarmState::LEADER_IDS.at(q_id - 1));
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "heuristic : %d", heur);
    return heur;
}

void Environment::GetSuccs(int sourceStateID, std::vector<int>* succIDs, 
                           std::vector<int>* costs){
    GetSuccs(sourceStateID, succIDs, costs, 0);
}

void Environment::GetSuccs(int sourceStateID, std::vector<int>* succIDs, 
                           std::vector<int>* costs, int q_id)
{
    assert(sourceStateID != GOAL_STATE);
    throw std::runtime_error("Shouldn't be calling this for lazy!");
}

void Environment::GetLazySuccs(int sourceStateID, std::vector<int>* succIDs, 
                        std::vector<int>* costs, std::vector<bool>* isTrueCost){
    int q_id = 0;
    GetLazySuccs(q_id, sourceStateID, succIDs, costs, isTrueCost);
}

void Environment::GetLazySuccs(int q_id, int sourceStateID, std::vector<int>* succIDs, 
                        std::vector<int>* costs, std::vector<bool>* isTrueCost)
{
    if (q_id == 0 && m_planner_type == PlannerType::MHA) {
        throw std::runtime_error("Expanding anchor is not implemented yet!");
    }
    // get the source_state
    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    // set the correct leader id.
    int leader_id;
    if (m_planner_type == PlannerType::MHA) {
        leader_id = SwarmState::LEADER_IDS.at(q_id -1);
        assert(leader_id == source_state->getLeader());
    } else {
        assert(q_id == 0);
        leader_id = source_state->getLeader();
    }

    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================", 
                    sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "expanding leader : %d, q_id : %d", leader_id, q_id);
    MPrimList current_mprims = m_mprims.getMotionPrims();
    succIDs->clear();
    succIDs->reserve(current_mprims.size());
    costs->clear();
    costs->reserve(current_mprims.size());

    // if we are expanding the start state, then no leaders would have been set
    // yet. So, we go ahead and expand it. The other condition to go ahead and
    // expand it is that the leader of the state is the current queue expanding it.

    // if (sourceStateID == m_start_state_id && m_planner_type==PlannerType::LAZYARA) {
    //     // must set the leader to whichever queue is expanding it and continue
    //     // to expand the state
    //     source_state->setLeader(0);
    // } 

    // debug and visualization
    // ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    // source_state->printToDebug(SEARCH_LOG);
    source_state->swarm_state().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->swarm_state().visualize();
        usleep(10000);
    }
    if (m_planner_type == PlannerType::Type::MHA) {
        for (auto mprim : current_mprims){
            GraphStatePtr successor;
            TransitionData t_data;
            
            bool generate_succ = generateAndSaveSuccessor(source_state, mprim,
                leader_id, successor, t_data);

            if(!generate_succ)
                continue;

            bool leader_change_required = 
            m_policy_generator->isLeaderChangeRequired(*source_state, *successor, leader_id, mprim);
            if (leader_change_required) {
                ROS_DEBUG_NAMED(SEARCH_LOG, "Leader change required to %d.", leader_id);
                successor->setLeader(leader_id);
                t_data.cost(t_data.cost() + m_param_catalog.m_motion_primitive_params.change_leader_cost);
            } else {
                successor->setLeader(source_state->getLeader());
            }
            successor->swarm_state().printToDebug(SEARCH_LOG);

            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_INFO("Found potential goal at: source->id %d, successor->id %d, "
                "cost: %d", source_state->id(), successor->id(), t_data.cost());
                succIDs->push_back(GOAL_STATE);
            } else {
                succIDs->push_back(successor->id());
            }
            ROS_DEBUG_NAMED(SEARCH_LOG, "cost : %d", t_data.cost());
            costs->push_back(t_data.cost());
            isTrueCost->push_back(true);

            if (leader_change_required) {
                // generate more successors for the original leader
                int old_leader_id = source_state->getLeader();
                GraphStatePtr other_successor;
                TransitionData other_tdata;
                bool other_succ_gen = generateAndSaveSuccessor(source_state,
                            mprim, old_leader_id, other_successor, other_tdata);
                if (!other_succ_gen)
                    continue;
                other_successor->setLeader(old_leader_id);
                if (m_goal->isSatisfiedBy(successor)){
                    m_goal->storeAsSolnState(successor);
                    ROS_INFO("Found potential goal at: source->id %d, successor->id %d, "
                    "cost: %d", source_state->id(), successor->id(), t_data.cost());
                    succIDs->push_back(GOAL_STATE);
                } else {
                    succIDs->push_back(other_successor->id());
                }
                costs->push_back(other_tdata.cost());
                isTrueCost->push_back(true);
                other_successor->swarm_state().printToDebug(SEARCH_LOG);
                ROS_DEBUG_NAMED(SEARCH_LOG, "cost : %d", other_tdata.cost());
            }
        }
    }
    
    if (m_planner_type == PlannerType::Type::LAZYARA) {
        for (int i = 0; i < NUM_LEADERS; i++) {
            int current_leader_id = SwarmState::LEADER_IDS.at(i);
            for (auto mprim : current_mprims){
                GraphStatePtr successor;
                TransitionData t_data;
                
                bool generate_succ = generateAndSaveSuccessor(source_state, mprim,
                    current_leader_id, successor, t_data);
                if(!generate_succ)
                    continue;
                if (current_leader_id != leader_id) {
                    t_data.cost(t_data.cost() + m_param_catalog.m_motion_primitive_params.change_leader_cost);
                }
                successor->setLeader(current_leader_id);
                if (m_goal->isSatisfiedBy(successor)){
                    m_goal->storeAsSolnState(successor);
                    ROS_INFO("Found potential goal at: source->id %d, successor->id %d, "
                    "cost: %d", source_state->id(), successor->id(), t_data.cost());
                    succIDs->push_back(GOAL_STATE);
                } else {
                    succIDs->push_back(successor->id());
                }
                costs->push_back(t_data.cost());
                isTrueCost->push_back(true);
            }
        }
    }
    ROS_DEBUG_NAMED(SEARCH_LOG, "size of succIDs : %lu", succIDs->size());
    ROS_DEBUG_NAMED(SEARCH_LOG, "size of costs : %lu", costs->size());
    std::cin.get();
}

bool Environment::generateAndSaveSuccessor(const GraphStatePtr source_state,
                                    MotionPrimitivePtr mprim,
                                    int leader_id,
                                    GraphStatePtr& successor,
                                    TransitionData& t_data)
{
    GraphStatePtr leader_moved_state;
    // Each MPrim is applied in 4 steps.

    // Step 1 : Apply the mprim motion to just the leader. This generates a
    // leader_moved_state
    if (!mprim->apply(*source_state, leader_id, leader_moved_state)){
        return false;
    }
    // if this is an adaptive primitive that succeeded, we should just skip
    // ahead to the end
    bool is_adaptive = (mprim->getPrimitiveType() == MPrim_Type::NAVAMP);
    if (!is_adaptive) {
        // Step 2 : For the policy, we assume that none of the followers have moved. We
        // take in the leader_moved_state and generate the policy, which gives
        // us the successor.
        if (!m_policy_generator->applyPolicy(*leader_moved_state, leader_id, successor)) {
            return false;
        }
        // Step 3 : compute the cost of the policy
        int policy_cost = m_policy_generator->computePolicyCost(*source_state,
            leader_id, successor);
        // Step 4 : compute the TData
        mprim->computeTData(*source_state, leader_id, successor, t_data);
        // ROS_DEBUG_NAMED(SEARCH_LOG, "policy_cost : %d", policy_cost);
        // We need to set the cost of the tData because the policyGenerator
        // is not aware of the cost of the mprim
        t_data.cost(mprim->getBaseCost() + policy_cost);

        if(!m_cspace_mgr->isValidSuccessor(*successor) ||
                    !m_cspace_mgr->isValidTransitionStates(t_data)) {
            return false;
        }
    } else {
        successor = leader_moved_state;
        mprim->computeTData(*source_state, leader_id, successor, t_data);
        t_data.cost(mprim->getBaseCost());
    }
    assert(successor != NULL);
    successor->printToDebug(SEARCH_LOG);

    // save the successor to the hash manager
    // generate the edge
    // push back to succIDs
    // push back to costs
    // set isTrueCost
    // add to edge cache
    // NOTE : You probably don't have to mess with the part below this as
    // long as you have the right successor in the `successor` variable.
    m_hash_mgr->save(successor);
    // ROS_DEBUG_NAMED(SEARCH_LOG,"Generated successor with id %d", successor->id());
    // Edge key;
    return true;
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int Environment::GetTrueCost(int parentID, int childID){
    throw std::runtime_error("not doing lazy at the moment!");
    return 0;
    // TransitionData t_data;

    // if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
    //   ROS_ERROR("transition hasn't been found between %d and %d??", parentID, childID);
    //     assert(false);
    // }
    // PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

    // ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
    // GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
    // GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
    // GraphStatePtr successor, tmp;
    // tmp = source_state;
    // // MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));
    // int total_cost = 0;
    // for (auto&& mprim : m_edges.at(Edge(parentID, childID))) {
    //     if (!mprim->apply(*tmp, successor, t_data)){
    //         return -1;
    //     }
    //     bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
    //                             m_cspace_mgr->isValidTransitionStates(t_data));
    //     if (!valid_successor){
    //         return -1;
    //     }
    //     total_cost += t_data.cost();
    //     tmp = successor;
    // }
    // // mprim->printEndCoord();
    // // mprim->print();
    // //source_state->printToInfo(SEARCH_LOG);
    // //successor->printToInfo(SEARCH_LOG);
    // successor->id(m_hash_mgr->getStateID(successor));

    // // right before this point, the successor's graph state does not match the
    // // stored robot state (because we modified the graph state without calling
    // // ik and all that). this call updates the stored robot pose.
    // real_next_successor->robot_pose(successor->robot_pose());

    // // bool matchesEndID = (successor->id() == childID) || (childID == GOAL_STATE);
    // // assert(matchesEndID);

    // return total_cost;
}

/**
 * @brief Transfers the state to the other queues. In this case, it's just
 * changing the leader id
 * 
 * @param transfer_q_id is the q_id that you want to transfer to; NOT the leader
 * @param extraCosts the cost of the transfer
 */
void Environment::TransferFunction(int transfer_q_id, const std::vector<int>& stateList,
                std::vector<int>* transferredList, std::vector<int>* extraCosts)
{
    if (transfer_q_id == 0)
        throw new std::runtime_error("Cannot use transfer function for anchor!");
    transferredList->clear();
    extraCosts->clear();
    transferredList->reserve(stateList.size());
    extraCosts->reserve(stateList.size());
    // change the leader to this.
    int leader_id = SwarmState::LEADER_IDS.at(transfer_q_id - 1);
    for (auto& stateID : stateList) {
        GraphStatePtr source_state = m_hash_mgr->getGraphState(stateID);
        GraphStatePtr transferred_state(new GraphState(*source_state));
        transferred_state->setLeader(leader_id);
        m_hash_mgr->save(transferred_state);
        transferredList->push_back(transferred_state->id());
        extraCosts->push_back(m_param_catalog.m_motion_primitive_params.change_leader_cost);
    }
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    m_edges.clear();

    auto swarm_start_pos = search_request->m_params->swarm_start;
    auto swarm_start = SwarmState::transformSwarmToPos(swarm_start_pos);
    if (!(static_cast<int>(swarm_start.robots_pose().size()) == SwarmState::NUM_ROBOTS))
        return false;

    if(!m_cspace_mgr->isValid(swarm_start)) {
        ROS_ERROR("Start state is invalid!");
        swarm_start.visualize();
        return false;
    }

    GraphStatePtr start_graph_state = std::make_shared<GraphState>(swarm_start);
    // HACK : setting the leader to the 0th leader arbitrarily
    start_graph_state->setLeader(SwarmState::LEADER_IDS.at(0));
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    m_start_state_id = start_id;
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    swarm_start.printToDebug(SEARCH_LOG);
    swarm_start.visualize();
    // std::cin.get();

    m_goal = search_request->createGoalState();
    if(!m_cspace_mgr->isValid(m_goal->getSwarmState())) {
        ROS_ERROR("Goal state is invalid!");
        m_goal->getSwarmState().visualize();
        return false;
    }

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    m_goal->getSwarmState().printToDebug(SEARCH_LOG);
    m_goal->getSwarmState().printContToDebug(SEARCH_LOG);
    m_goal->getSwarmState().visualize();
    // std::cin.get();

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);
    m_heur_mgr->printSummaryToDebug(HEUR_LOG);
    NavAdaptiveMotionPrimitive::setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = std::make_shared<GraphState>(*start_graph_state);
    // put all the robots in the swarm at 0. That should be fake enough.
    SwarmState fake_swarm_state = fake_goal->swarm_state();
    auto fake_robots_list = fake_swarm_state.robots_pose();
    for (auto& robot_state : fake_robots_list) {
        DiscRobotState fake_d_state = robot_state.getDiscRobotState();
        fake_d_state.x(0); fake_d_state.y(0);
        robot_state = RobotState(fake_d_state);
    }
    fake_swarm_state = SwarmState(fake_robots_list);
    fake_goal->swarm_state(fake_swarm_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    assert(goal_id == GOAL_STATE);
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params);

    SwarmState::configureSwarmState(m_param_catalog.m_swarm_description_params);

    m_heur_mgr->initializeHeuristics();

    m_cspace_mgr = std::make_shared<CollisionSpaceMgr>(m_param_catalog.m_robot_description_params);
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);

    // set mprim params
    m_mprims.setMprimParams(m_param_catalog.m_motion_primitive_params);
    // load up motion primitives
    m_mprims.loadMPrims();

    m_policy_generator.reset(new PolicyGenerator(m_cspace_mgr, 
                                m_param_catalog.m_robot_description_params));

    // load up static pviz instance for visualizations. 
    Visualizer::createSwarmVizInstance(m_nodehandle, std::string("/map"));
    Visualizer::configureRobotParams(m_param_catalog.m_robot_description_params);
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    return;
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
std::vector<SwarmState> Environment::reconstructPath(std::vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr, m_policy_generator);
    std::vector<SwarmState> final_path = postprocessor.reconstructPath(soln_path, *m_goal,
        m_edges,
        m_num_leader_changes);
    if(m_param_catalog.m_visualization_params.final_path){
        ROS_DEBUG_NAMED(SEARCH_LOG, "Visualizing final path");
        std::cin.get();
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}

void Environment::generateStartState(SearchRequestPtr search_request) {
    // TODO: implement this;
    return;
    // ContObjectState start_obj_state(search_request->m_params->obj_start);
    // ContBaseState base_start(search_request->m_params->base_start);
    // RobotState start_robot_state(base_start, start_obj_state);
    // start_robot_state.visualize();
    // m_cspace_mgr->visualizeAttachedObject(start_robot_state);
    // ROS_DEBUG_NAMED(CONFIG_LOG, "Generate start state : Keyboard");
    // // std::cin.get();
    // search_request->m_params->base_start = start_robot_state.getContBaseState();
    // search_request->m_params->right_arm_start = start_robot_state.right_arm();
    // search_request->m_params->left_arm_start = start_robot_state.left_arm();
}
