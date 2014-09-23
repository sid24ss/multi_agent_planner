#include <multi_agent_planner/PathPostProcessor.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/Visualizer.h>


using namespace multi_agent_planner;

PathPostProcessor::PathPostProcessor(HashManagerPtr hash_mgr,
                                    CSpaceMgrPtr cspace_mgr,
                                    PolicyGeneratorPtr policy_generator)
    :   m_cspace_mgr(cspace_mgr),
        m_hash_mgr(hash_mgr),
        m_policy_generator(policy_generator)
{ }


/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
std::vector<SwarmState> PathPostProcessor::reconstructPath(
                                            std::vector<int> soln_path,
                                            GoalState& goal_state,
                                            std::map< std::pair<int,int>,
                                            MotionPrimitivePtr>& edge_cache,
                                            int& num_leader_changes)
{
    double temptime = clock();
    num_leader_changes = 0;
    std::vector<TransitionData> transition_states;
    // the last state in the soln path return by the SBPL planner will always be
    // the goal state ID. Since this doesn't actually correspond to a real state
    // in the heap, we have to look it up.
    GraphStatePtr soln_state = goal_state.getSolnState();
    for (size_t i=0; i < soln_path.size()-1; i++){
        // MotionPrimitivePtr mprim = edge_cache.at(std::make_pair(soln_path[i],
            // soln_path[i+1]));
        // TransitionData best_transition;
        GraphStatePtr source_state = m_hash_mgr->getGraphState(soln_path[i]);
        source_state->swarm_state().visualize();
        if ((i+1) == soln_path.size() - 1)
            soln_path.back() = soln_state->id();
        // GraphStatePtr successor, leader_moved_state;
        GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(soln_path[i+1]);
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "reconstructing %d - %d",
                                source_state->id(), real_next_successor->id());
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "source (%d) : ", source_state->id());
        source_state->printToDebug(POSTPROCESSOR_LOG);
        // bool success = false;

        TransitionData t_data;
        // get the leader
        int leader_id = source_state->swarm_state().getLeader();
        // if ((i + 1) == soln_path.size() -1) {
        //     leader_id = source_state->swarm_state().getLeader();
        //     mprim->apply(*source_state, leader_id, real_next_successor);
        //     real_next_successor->id(m_hash_mgr->getStateID(real_next_successor));
        // } else {
        //     leader_id = real_next_successor->swarm_state().getLeader();
        // }
        // leader_id = real_next_successor->swarm_state().getLeader();
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "real_next_successor (%d) : ", real_next_successor->id());
        if(source_state->getLeader() != real_next_successor->getLeader())
            num_leader_changes++;
        real_next_successor->printToDebug(POSTPROCESSOR_LOG);
        // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "expanding with leader : %d", leader_id);
        // bool success = mprim->apply(*source_state, leader_id, leader_moved_state);
        // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "leader_moved_state:");
        // leader_moved_state->printToDebug(POSTPROCESSOR_LOG);
        // skip policy if adaptive
        // bool is_adaptive = (mprim->getPrimitiveType() == MPrim_Type::NAVAMP);
        // if (is_adaptive){
        //     ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "ADAPTIVE MOTION!");
        //     mprim->apply(*source_state, leader_id, real_next_successor);
        //     real_next_successor->id(m_hash_mgr->getStateID(real_next_successor));
        // } else {
            // success = mprim->apply(*source_state, leader_id, leader_moved_state);
            // success = success && m_policy_generator->applyPolicy(
                                    // *leader_moved_state, leader_id, successor);
            // successor->setLeader(leader_id);
        // }
        MotionPrimitive::computeTData(*source_state, leader_id, real_next_successor, t_data);
        // mprim->computeTData(*source_state, leader_id, successor, t_data);
        // ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "successor:");
        // successor->printToDebug(POSTPROCESSOR_LOG);
        transition_states.push_back(t_data);
        
        // if (success) {
        //     transition_states.push_back(t_data);
        // } else {
        //     ROS_ERROR("Successor not found during path reconstruction!");
        // }
        // successor->id(m_hash_mgr->getStateID(successor));
        // change the last state in the path to the actual state
        // if ((i + 1) == soln_path.size() -1) {
        //     soln_path[i+1] = real_next_successor->id();
        // }
        // the successor's id for the goal state is not going to match with the
        // goal state ID (which is in real_next_successor->id())
        // bool matchesEndID = (successor->id() == real_next_successor->id())
        //                     || ((i+1) == soln_path.size()-1);
        // assert(matchesEndID);
    }
    
    ROS_INFO("Finding best transition took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    // std::vector<SwarmState> final_path = getFinalPath(soln_path, 
    //                                                 transition_states,
    //                                                 goal_state);

    // temptime = clock();
    ROS_INFO_NAMED(POSTPROCESSOR_LOG, "Number of leader changes in the path: %d",
        num_leader_changes);
    std::vector<SwarmState> final_path = getFinalPath(soln_path,
                                                transition_states, goal_state);
    // ROS_INFO("Shortcutting took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    return final_path;
}

void PathPostProcessor::visualizeFinalPath(std::vector<SwarmState> path) {
    // path is a vector of SwarmStates; state is a SwarmState
    for (auto& state : path){
        state.visualize();
        usleep(10000);
    }
}

/*! \brief Retrieve the final path, with intermediate states and all. There's
 * some slight funny business with TransitionData for intermediate base states
 * because they're not captured in the RobotState. Instead, they're held
 * separately in TransitionData ONLY if the motion primitive has the base
 * moving. There's one more state_id than transition state.
 * */
std::vector<SwarmState> PathPostProcessor::getFinalPath(const std::vector<int>& state_ids,
                                 const std::vector<TransitionData>& transition_states,
                                 GoalState& goal_state){
    std::vector<SwarmState> swarm_states;
    for (size_t i=0; i < transition_states.size(); i++){
        // throw in the first point in this transition
        GraphStatePtr source_state = m_hash_mgr->getGraphState(state_ids[i]);
        swarm_states.push_back(source_state->swarm_state());
        // throw in all the interm steps
        for (auto& interm_swarm_step : transition_states[i].interm_swarm_steps())
        {
            swarm_states.push_back(interm_swarm_step);
        }
    }
    { // throw in the last point
        GraphStatePtr soln_state = m_hash_mgr->getGraphState(state_ids.back());
        swarm_states.push_back(soln_state->swarm_state());
    }
    return swarm_states;
}
