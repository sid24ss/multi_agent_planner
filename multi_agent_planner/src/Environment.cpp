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
    // need to do this because planner starts queues from 0 (anchor), 1 ... N
    // int leader_id = SwarmState::LEADER_IDS[q_id];

    // ROS_DEBUG_NAMED(SEARCH_LOG, 
    //         "==================Expanding state %d==================", 
    //                 sourceStateID);
    // succIDs->clear();
    // succIDs->reserve(m_mprims.getMotionPrims().size());
    // costs->clear();
    // costs->reserve(m_mprims.getMotionPrims().size());

    // GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    // ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    // source_state->swarm_state().printToDebug(SEARCH_LOG);
    // if (m_param_catalog.m_visualization_params.expansions) {
    //     SwarmState exp_swarm = source_state->swarm_state();
    //     exp_swarm.visualize();
    //     usleep(5000);
    // }
    // for (auto mprim : m_mprims.getMotionPrims()) {
    //     // ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
    //     // mprim->printEndCoord();
    //     GraphStatePtr successor;
    //     TransitionData t_data;
    //     if (!mprim->apply(*source_state, leader_id, successor, t_data)) {
    //         ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
    //         continue;
    //     }

    //     if (m_cspace_mgr->isValidSuccessor(*successor) &&
    //         m_cspace_mgr->isValidTransitionStates(t_data)){
    //         ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    //         source_state->printToDebug(SEARCH_LOG);
    //         m_hash_mgr->save(successor);
    //         ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
    //                         successor->id());
    //         successor->printToDebug(MPRIM_LOG);

    //         if (m_goal->isSatisfiedBy(successor)){
    //             m_goal->storeAsSolnState(successor);
    //             ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
    //                 mprim->cost());
    //             succIDs->push_back(GOAL_STATE);
    //         } else {
    //             succIDs->push_back(successor->id());
    //         }
    //         costs->push_back(mprim->cost());
    //         ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
    //     } else {
    //         //successor->robot_pose().visualize();
    //         ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
    //     }
    // }
}

void Environment::GetLazySuccs(int sourceStateID, std::vector<int>* succIDs, 
                        std::vector<int>* costs, std::vector<bool>* isTrueCost){
    int q_id = 0;
    GetLazySuccs(sourceStateID, succIDs, costs, isTrueCost, q_id);
}

void Environment::GetLazySuccs(int sourceStateID, std::vector<int>* succIDs, 
                        std::vector<int>* costs, std::vector<bool>* isTrueCost,
                        int q_id)
{
    if (q_id == 0) {
        ROS_ERROR("Expanding anchor! Not implemented yet");
        std::cin.get();
    }
    // set the correct leader id.
    int leader_id = SwarmState::LEADER_IDS.at(q_id -1);


    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================", 
                    sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "expanding leader : %d", leader_id);
    MPrimList current_mprims = m_mprims.getMotionPrims();
    succIDs->clear();
    succIDs->reserve(current_mprims.size());
    costs->clear();
    costs->reserve(current_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    // set who the leader is.
    source_state->setLeader(leader_id);

    // debug and visualization
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->swarm_state().printToDebug(SEARCH_LOG);
    source_state->swarm_state().printContToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->swarm_state().visualize();
        usleep(10000);
    }

    for (auto mprim : current_mprims){
        GraphStatePtr successor, leader_moved_state;
        TransitionData t_data;
        // Each MPrim is applied in 4 steps.

        // Step 1 : Apply the mprim motion to just the leader. This generates a
        // leader_moved_state
        if (!mprim->apply(*source_state, leader_id, leader_moved_state)){
            continue;
        }
        // if this is an adaptive primitive that succeeded, we should just skip
        // ahead to the end
        bool is_adaptive = (mprim->getPrimitiveType() == MPrim_Type::NAVAMP);
        if (!is_adaptive) {
            // Step 2 : For the policy, we assume that none of the followers have moved. We
            // take in the leader_moved_state and generate the policy, which gives
            // us the successor.
            if (!m_policy_generator->applyPolicy(*leader_moved_state, leader_id, successor,
                mprim->getDisplacement())) {
                continue;
            }
            // Step 3 : compute the cost of the policy
            int policy_cost = m_policy_generator->computePolicyCost(*source_state,
                leader_id, successor);
            // Step 4 : compute the TData
            mprim->computeTData(*source_state, leader_id, successor, t_data);
            // We need to set the cost of the tData because the policyGenerator
            // is not aware of the cost of the mprim
            t_data.cost(mprim->getBaseCost() + policy_cost);

            if(!m_cspace_mgr->isValidSuccessor(*successor) ||
                        !m_cspace_mgr->isValidTransitionStates(t_data)) {
                continue;
            }
        } else {
            successor = leader_moved_state;
            mprim->computeTData(*source_state, leader_id, successor, t_data);
            t_data.cost(mprim->getBaseCost());
        }
        m_hash_mgr->save(successor);

        successor->swarm_state().printToDebug(SEARCH_LOG);
        successor->swarm_state().printContToDebug(SEARCH_LOG);

        Edge key;
        if (m_goal->isSatisfiedBy(successor)){
          m_goal->storeAsSolnState(successor);
          ROS_INFO("Found potential goal at: source->id %d, successor->id %d, "
            "cost: %d", source_state->id(), successor->id(), t_data.cost());
          succIDs->push_back(GOAL_STATE);
          key = Edge(sourceStateID, GOAL_STATE);
        } else {
          succIDs->push_back(successor->id());
          key = Edge(sourceStateID, successor->id());
        }
        costs->push_back(t_data.cost());
        // ROS_DEBUG_NAMED(SEARCH_LOG, "cost to successor : %d", t_data.cost());
        isTrueCost->push_back(true);

        m_edges.insert(std::map<Edge, MotionPrimitivePtr>::value_type(key, mprim));
    }
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
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    swarm_start.printToDebug(SEARCH_LOG);
    swarm_start.visualize();
    std::cin.get();

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

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);
    m_heur_mgr->printSummaryToDebug(HEUR_LOG);
    NavAdaptiveMotionPrimitive::setGoal(*m_goal);
    std::cin.get();

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
        m_edges);
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
