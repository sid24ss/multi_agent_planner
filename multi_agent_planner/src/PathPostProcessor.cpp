#include <multi_agent_planner/PathPostProcessor.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/Visualizer.h>


using namespace multi_agent_planner;

PathPostProcessor::PathPostProcessor(HashManagerPtr hash_mgr, CSpaceMgrPtr cspace_mgr):
m_cspace_mgr(cspace_mgr), m_hash_mgr(hash_mgr)
{
}


/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
std::vector<SwarmState> PathPostProcessor::reconstructPath(
                                            std::vector<int> soln_path,
                                            GoalState& goal_state,
                                            std::map< std::pair<int,int>,
                                            MotionPrimitivePtr>& edge_cache)
{
    double temptime = clock();
    std::vector<TransitionData> transition_states;
    // the last state in the soln path return by the SBPL planner will always be
    // the goal state ID. Since this doesn't actually correspond to a real state
    // in the heap, we have to look it up.
    for (size_t i=0; i < soln_path.size()-1; i++){
        MotionPrimitivePtr mprim = edge_cache.at(std::make_pair(soln_path[i],
            soln_path[i+1]));
        // TransitionData best_transition;
        GraphStatePtr source_state = m_hash_mgr->getGraphState(soln_path[i]);
        source_state->swarm_state().visualize();
        GraphStatePtr successor;
        GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(soln_path[i+1]);
        ROS_DEBUG_NAMED(POSTPROCESSOR_LOG, "reconstructing %d - %d",
                                source_state->id(), real_next_successor->id());
        TransitionData t_data;
        // get the leader
        int leader_id = source_state->swarm_state().getLeader();
        bool success = mprim->apply(*source_state, leader_id, successor, t_data);
        if (success) {
            transition_states.push_back(t_data);
        } else {
            ROS_ERROR("Successor not found during path reconstruction!");
        }
        successor->id(m_hash_mgr->getStateID(successor));
        // change the last state in the path to the actual state
        if ((i + 1) == soln_path.size() -1) {
            soln_path[i+1] = successor->id();
        }
        // the successor's id for the goal state is not going to match with the
        // goal state ID (which is in real_next_successor->id())
        bool matchesEndID = (successor->id() == real_next_successor->id())
                            || ((i+1) == soln_path.size()-1);
        assert(matchesEndID);
    }
    
    ROS_INFO("Finding best transition took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    // std::vector<SwarmState> final_path = getFinalPath(soln_path, 
    //                                                 transition_states,
    //                                                 goal_state);

    // temptime = clock();
    std::vector<SwarmState> final_path = getFinalPath(soln_path,
                                                transition_states, goal_state);
    // ROS_INFO("Shortcutting took %.3f", (clock()-temptime)/(double)CLOCKS_PER_SEC);
    return final_path;
}

void PathPostProcessor::visualizeFinalPath(std::vector<SwarmState> path) {
    // path is a vector of SwarmStates; state is a SwarmState
    for (auto& state : path){
        state.visualize();
        usleep(50000);
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
