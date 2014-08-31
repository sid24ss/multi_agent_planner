#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <cassert>

#define GOAL_STATE 1
using namespace monolithic_pr2_planner;
using namespace boost;

// stateid2mapping pointer inherited from sbpl interface. needed for planner.
Environment::Environment(ros::NodeHandle nh)
    :   m_hash_mgr(new HashManager(&StateID2IndexMapping)),
        m_nodehandle(nh), m_mprims(m_goal),
        m_heur_mgr(new HeuristicMgr()),
        m_planner_type(T_SMHA) {
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

    // Fetch params again, in case they're being modified between calls.
    // m_param_catalog.fetch(m_nodehandle);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
void Environment::setPlannerType(int planner_type) {
    m_planner_type = planner_type;
    m_heur_mgr->setPlannerType(planner_type);
    ROS_INFO_NAMED(SEARCH_LOG, "Setting planner type: %d", m_planner_type);
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id) {
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(
        search_request_params));
    configureQuerySpecificParams(search_request);
    if(search_request->m_params->underspecified_start) {
        ROS_DEBUG_NAMED(CONFIG_LOG, "underspecified_start. Will generate start state.");
        generateStartState(search_request);
    }
    if (!setStartGoal(search_request, start_id, goal_id)) {
        return false;
    }

    return true;
}

int Environment::GetGoalHeuristic(int stateID) {
    // For now, return the max of all the heuristics
    return GetGoalHeuristic(0, stateID);
}

int Environment::GetGoalHeuristic(int heuristic_id, int stateID) {
    GraphStatePtr successor = m_hash_mgr->getGraphState(stateID);
    if(m_goal->isSatisfiedBy(successor) || stateID == GOAL_STATE){
        return 0;
    }
    std::unique_ptr<stringintmap> values;
    m_heur_mgr->getGoalHeuristic(successor, values);
    
    for (auto& heur : (*values)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "%s : %d", heur.first.c_str(), heur.second);
    }

    // heuristic_id = 3;

    if(!m_use_new_heuristics){
      switch (heuristic_id) {
        case 0:  // Anchor
          return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
        case 1:  // base 1
          return values->at("base_with_rot_0");
        case 2:  // Base1, Base2 heur
          return values->at("base_with_rot_door");
        case 3:
          return values->at("base_with_rot_0");
        case 4:
          return values->at("admissible_endeff");
      }
    }
    else{
      double w_bfsRot = 0.5;
      double w_armFold = 0.5;

      switch (heuristic_id) {
        case 0:  // Anchor
          return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
        //case 1:  // ARA Heur 
          //return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
        case 1:  // Base1, Base2 heur
          return static_cast<int>(0.5*(*values).at("base_with_rot_0") + 0.5*(*values).at("endeff_rot_goal"));
        case 2:  // Base1, Base2 heur
          return static_cast<int>(1.0*(*values).at("base_with_rot_0") + 0.0*(*values).at("endeff_rot_goal"));
        case 3:
          if((*values).at("bfsRotFoot0")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot0") + w_armFold*(*values).at("arm_angles_folded"));
        case 4:
          if((*values).at("bfsRotFoot1")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot1") + w_armFold*(*values).at("arm_angles_folded"));
        case 5:
          if((*values).at("bfsRotFoot2")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot2") + w_armFold*(*values).at("arm_angles_folded"));
        case 6:
          if((*values).at("bfsRotFoot3")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot3") + w_armFold*(*values).at("arm_angles_folded"));
        case 7:
          if((*values).at("bfsRotFoot4")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot4") + w_armFold*(*values).at("arm_angles_folded"));
        case 8:
          if((*values).at("bfsRotFoot5")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot5") + w_armFold*(*values).at("arm_angles_folded"));
        case 9:
          if((*values).at("bfsRotFoot6")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot6") + w_armFold*(*values).at("arm_angles_folded"));
        case 10:
          if((*values).at("bfsRotFoot7")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot7") + w_armFold*(*values).at("arm_angles_folded"));
        case 11:
          if((*values).at("bfsRotFoot8")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot8") + w_armFold*(*values).at("arm_angles_folded"));
        case 12:
          if((*values).at("bfsRotFoot9")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot9") + w_armFold*(*values).at("arm_angles_folded"));
        case 13:
          if((*values).at("bfsRotFoot10")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot10") + w_armFold*(*values).at("arm_angles_folded"));
        case 14:
          if((*values).at("bfsRotFoot11")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot11") + w_armFold*(*values).at("arm_angles_folded"));
        case 15:
          if((*values).at("bfsRotFoot12")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot12") + w_armFold*(*values).at("arm_angles_folded"));
        case 16:
          if((*values).at("bfsRotFoot13")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot13") + w_armFold*(*values).at("arm_angles_folded"));
        case 17:
          if((*values).at("bfsRotFoot14")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot14") + w_armFold*(*values).at("arm_angles_folded"));
        case 18:
          if((*values).at("bfsRotFoot15")==0)
            return 0;
          return static_cast<int>(w_bfsRot*(*values).at("bfsRotFoot15") + w_armFold*(*values).at("arm_angles_folded"));
      }
    }

    /*
    switch (heuristic_id) {
      case 0:  // Anchor
        return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
      case 1:  // Base1
        return static_cast<int>(values->at("base_with_rot_0"));
      case 2:  // Base2
        return static_cast<int>(values->at("base_with_rot_door"));
      case 3:
        return static_cast<int>(values->at("admissible_endeff"));
    }
    */

    // switch (heuristic_id) {
    //   case 0:  // Anchor
    //     return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
    //   case 1:  // ARA Heur 
    //     return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
    //   case 2:  // Base1, Base2 heur
    //     return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("admissible_endeff"));
    //   case 3:
    //     return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("admissible_endeff"));
    // }

    /*
    switch (m_planner_type) {
        case T_SMHA:
        case T_MHG_REEX:
        case T_MHG_NO_REEX:
        case T_ARA:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // ARA Heur
                    return EPS2*std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                // case 2:
                //     return EPS2*(*values).at("admissible_endeff");
                case 2:  // Base1, Base2 heur
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_0") +
                        0.5f*(*values).at("endeff_rot_goal"));
                case 3:
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_door") +
                        0.5f*(*values).at("endeff_rot_goal"));
            }
            break;
        case T_IMHA:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // ARA Heur
                    return EPS2*std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 2:  // Base1, Base2 heur
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_0") + 0.5f*(*values).at("endeff_rot_goal"));
                case 3:
                    return static_cast<int>(0.5f*(*values).at("base_with_rot_door") + 0.5f*(*values).at("endeff_rot_goal"));
            }
            break;
        case T_MPWA:
            return (heuristic_id+1)*(EPS1*EPS2/NUM_SMHA_HEUR)*std::max(
                (*values).at("admissible_endeff"), (*values).at("admissible_base"));
            break;
        case T_EES:
            switch (heuristic_id) {
                case 0:  // Anchor
                    return std::max((*values).at("admissible_endeff"), (*values).at("admissible_base"));
                case 1:  // Inadmissible
                    return (*values).at("base_with_rot_0") + (*values).at("admissible_endeff");
                    // return static_cast<int>(0.5f*(*values)[4] + 0.5f*(*values).at("admissible_endeff"));
                case 2:  // Distance function
                    // return (*values)[2];
                    // ROS_DEBUG_NAMED(HEUR_LOG, "Arm : %d, Base : %d", (*values)[2],
                    //     (*values)[3]);
                    return (*values)["uniform_2d"] + (*values)["uniform_3d"];
            }
            break;
    }
    */

    // Post-paper
    // switch(heuristic_id){
    //     case 0: //Anchor
    //         return std::max(values.at("admissible_endeff"), values.at("admissible_base"));
    //     case 1: // base
    //         return values[2];
    //     case 2: // Base + arm
    //         return static_cast<int>(0.5f*values[2] +
    //             0.5f*values[0]);
    // }


    // ROS_DEBUG_NAMED(HEUR_LOG, "2: %d,\t 3: %d", values[2],
    //     values[3]);
    // EES

    return std::max((*values).at("admissible_base"), (*values).at("admissible_endeff"));
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    GetSuccs(sourceStateID, succIDs, costs, 0);
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, int ii)
{
    assert(sourceStateID != GOAL_STATE);

    ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if (m_param_catalog.m_visualization_params.expansions) {
        RobotState expansion_pose = source_state->robot_pose();
        expansion_pose.visualize(250/NUM_SMHA_HEUR*ii);
        // source_state->robot_pose().visualize(250/NUM_SMHA_HEUR*ii);
        m_cspace_mgr->visualizeAttachedObject(expansion_pose, 250/NUM_SMHA_HEUR*ii);
        // m_cspace_mgr->visualizeCollisionModel(expansion_pose);
        usleep(5000);
    }
    for (auto mprim : m_mprims.getMotionPrims()) {
        ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        // mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;
        if (!mprim->apply(*source_state, successor, t_data)) {
            ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
            continue;
        }

        if (m_cspace_mgr->isValidSuccessor(*successor,t_data) &&
            m_cspace_mgr->isValidTransitionStates(t_data)){
            ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
            source_state->printToDebug(SEARCH_LOG);
            m_hash_mgr->save(successor);
            ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
                            successor->id());
            successor->printToDebug(MPRIM_LOG);

            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d", successor->id(),
                    mprim->cost());
                succIDs->push_back(GOAL_STATE);
            } else {
                succIDs->push_back(successor->id());
            }
            costs->push_back(mprim->cost());
            ROS_DEBUG_NAMED(SEARCH_LOG, "motion succeeded with cost %d", mprim->cost());
        } else {
            //successor->robot_pose().visualize();
            ROS_DEBUG_NAMED(SEARCH_LOG, "successor failed collision checking");
        }
    }
}

void Environment::GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                        vector<int>* costs, std::vector<bool>* isTrueCost){
    int q_id = 0;
    GetLazySuccs(sourceStateID, succIDs, costs, isTrueCost, q_id);
}

void Environment::GetLazySuccs(int sourceStateID, vector<int>* succIDs, 
                        vector<int>* costs, std::vector<bool>* isTrueCost,
                        int q_id)
{
    // q_id = 4;
    ROS_DEBUG_NAMED(SEARCH_LOG, "expanding queue : %d", q_id);
    std::vector<MotionPrimitivePtr> current_mprims;
    switch(m_action_partition.at(q_id)){
        case PlanningModes::BASE_ONLY:
            current_mprims = m_mprims.getBaseAndTorsoMotionPrims();
            break;
        case PlanningModes::RIGHT_ARM:
            current_mprims = m_mprims.getArmMotionPrims();
            break;
        case PlanningModes::RIGHT_ARM_MOBILE:
        default:
            current_mprims = m_mprims.getMotionPrims();
    }

    ROS_DEBUG_NAMED(SEARCH_LOG, "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(current_mprims.size());
    costs->clear();
    costs->reserve(current_mprims.size());

    GraphStatePtr source_state = m_hash_mgr->getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    if(m_param_catalog.m_visualization_params.expansions){
        source_state->robot_pose().visualize(250 / NUM_SMHA_HEUR * q_id);
        usleep(10000);
    }

    GraphStatePtr policy_applied_state;
    std::vector<MotionPrimitivePtr> edge_prims;
    int policy_cost = 0;
    bool apply_only_policy = false;

    /*
     * pre-computation for arm mprims to get the base policy
     * the logic is to find the best parent. And iterate through the base prims
     * to find one that matches. If nothing matches, we will use the one with 
     * the least heuristic.
     * The problem is that I have to use the base mprims to generate the
     * sucessors lazily, because otherwise, when I evaluate the state,
     * it's going to be invalid if it can't be generated by the mprims.
     */

    // TODO : An issue is that we might end up with an invalid state because of
    // the policy. How do we prevent this? Maybe we can collission check only
    // the policy applied state, and if it isn't valid, use the source state itself.
    if (m_action_partition.at(q_id) == PlanningModes::RIGHT_ARM) {
        // get the base state of the successor
        int base_x = source_state->base_x();
        int base_y = source_state->base_y();
        std::pair <int,int> best_parent = m_heur_mgr->getBestParent(
            "base_with_rot_0", base_x, base_y);
        std::vector<MotionPrimitivePtr> base_mprims = m_mprims.getBaseAndTorsoMotionPrims();
        ROS_DEBUG_NAMED(SEARCH_LOG, "source state : %d, %d best heur dir: %d %d",
            base_x, base_y, best_parent.first, best_parent.second);
        
        bool matching_base_mprim_found = false;

        // vector to store the successful prims
        std::vector< std::pair<GraphStatePtr, MotionPrimitivePtr> > base_successors;

        // iterate through and find one that matches
        for (int i = 0; i < static_cast<int>(base_mprims.size()) 
                            && !matching_base_mprim_found; ++i) {
            // apply to get a new state
            GraphStatePtr base_successor;
            TransitionData t_data;
            if (!base_mprims[i]->apply(*source_state, base_successor, t_data))
                continue;
            // unsure if required. but let's do it anyway so that it doesn't
            // break anything downstream
            base_successor->id(source_state->id());
            // I have a base successor. Time to check if it matches with our
            // best parent.
            if (best_parent.first == base_successor->base_x() 
                && best_parent.second == base_successor->base_y()) {
                if (!m_cspace_mgr->isValidSuccessor(*base_successor, t_data))
                    continue;
                matching_base_mprim_found = true;
                ROS_DEBUG_NAMED(SEARCH_LOG, "matching base prim found!");
                policy_applied_state = base_successor;
                edge_prims.clear();
                edge_prims.push_back(base_mprims[i]);
                policy_cost = base_mprims[i]->cost();
            }
            // whether it does or not, save it to the list of successors.
            base_successors.push_back(std::make_pair(base_successor, base_mprims
                [i]));
        }
        if (!matching_base_mprim_found) {
            // too bad. Let's settle for the one that minimizes the heuristic
            // anyway.
            int best_heuristic = INFINITECOST;
            for (auto state_prim_pair : base_successors) {
                int heur = m_heur_mgr->getGoalHeuristic(state_prim_pair.first, "base_with_rot_0");
                if (heur < best_heuristic) {
                    TransitionData t_data;
                    t_data.motion_type(MPrim_Types::BASE);
                    if (!m_cspace_mgr->isValidSuccessor(*state_prim_pair.first, t_data))
                        continue;
                    best_heuristic = heur;
                    policy_applied_state = state_prim_pair.first;
                    edge_prims.clear();
                    edge_prims.push_back(state_prim_pair.second);
                    policy_cost = state_prim_pair.second->cost();
                }
            }
            if (best_heuristic == INFINITECOST)
                policy_applied_state.reset(new GraphState(*source_state));
        }
        // source_state->robot_pose().visualize(250 / NUM_SMHA_HEUR * q_id);
        // policy_applied_state->robot_pose().visualize(0);
        // std::cin.get();
    } else if (m_action_partition.at(q_id) == PlanningModes::BASE_ONLY) {
        // BASE queues; need to apply arm policies. Can do something smart-ish
        // here?
        // if the heuristic is not zero yet, tuck it in.
        int heur;
        if (q_id == 1 || q_id == 3) {
            heur = m_heur_mgr->getGoalHeuristic(source_state, "base_with_rot_0");
        } else {
            heur = m_heur_mgr->getGoalHeuristic(source_state, "base_with_rot_door");
        }
        bool is_tucked_in = m_heur_mgr->isArmTuckedIn(source_state);
        // if (heur > 0) {
        if (q_id == 3) {
            if (is_tucked_in) {
                // ROS_DEBUG_NAMED(SEARCH_LOG, "Heur boundary reached and arm is tucked in");
                // std::cin.get();
                // need to untuck first
                MotionPrimitivePtr untuck_arm_prim = m_mprims.getUntuckArmPrim(true);
                MotionPrimitivePtr untuck_arm_prim_part = m_mprims.getUntuckArmPrim(false);
                TransitionData t_data;
                if (untuck_arm_prim->apply(*source_state, policy_applied_state,t_data)){
                    edge_prims.clear();
                    edge_prims.push_back(untuck_arm_prim);
                    policy_cost = untuck_arm_prim->cost();
                } else if (untuck_arm_prim_part->apply(*source_state, policy_applied_state,t_data)) {
                    edge_prims.clear();
                    edge_prims.push_back(untuck_arm_prim_part);
                    policy_cost = untuck_arm_prim_part->cost();
                } else {
                    policy_applied_state.reset(new GraphState(*source_state));
                }
            } else {
                policy_applied_state.reset(new GraphState(*source_state));
                // policy here is to find some mprim that has a lower heuristic
                // than the current state. We break when we find just one that
                // has a lower heuristic.
                // int current_arm_heur = m_heur_mgr->getGoalHeuristic(source_state, "admissible_endeff");
                std::vector<MotionPrimitivePtr> arm_mprims = m_mprims.getArmMotionPrims();
                for (auto& prim : arm_mprims) {
                    GraphStatePtr arm_policy_applied;
                    arm_policy_applied.reset(new GraphState(*source_state));
                    arm_policy_applied->lazyApplyMPrim(prim->getEndCoord());
                    ContObjectState obj_state_rel_map = arm_policy_applied->getObjectStateRelMapFromState();
                    ContObjectState current_obj_state = source_state->getObjectStateRelMapFromState();
                    if (ContObjectState::distance(obj_state_rel_map, m_goal->getObjectState())
                         < ContObjectState::distance(current_obj_state, m_goal->getObjectState()))
                    {
                        ROS_DEBUG_NAMED(SEARCH_LOG, "Applying a straight-line arm policy.");
                        // bam! we have a policy that reduces the distance
                        TransitionData t_data;
                        if (!prim->apply(*source_state, policy_applied_state, t_data))
                            continue;
                        // policy_applied_state = arm_policy_applied;
                        edge_prims.clear();
                        edge_prims.push_back(prim);
                        policy_cost = prim->cost();
                        if (heur == 0) {
                            ROS_DEBUG_NAMED(SEARCH_LOG, "setting only policy");
                            // std::cin.get();
                            apply_only_policy = true;
                        }
                        break;
                    }
                }
            }
        } else {  // if it is q_id 1 or 2, just tuck in
            if (!is_tucked_in){
                // need to tuck arm
                MotionPrimitivePtr tuck_arm_prim = m_mprims.getTuckArmPrim();
                TransitionData t_data;
                if(!tuck_arm_prim->apply(*source_state, policy_applied_state, t_data)) {
                    policy_applied_state.reset(new GraphState(*source_state));
                } else {
                    edge_prims.clear();
                    edge_prims.push_back(tuck_arm_prim);
                    policy_cost = tuck_arm_prim->cost();
                }
            } else {
                policy_applied_state.reset(new GraphState(*source_state));
            }
        }
        // } else {
        //     if (is_tucked_in) {
        //         ROS_DEBUG_NAMED(SEARCH_LOG, "Heur boundary reached and arm is tucked in");
        //         // std::cin.get();
        //         // need to untuck first
        //         MotionPrimitivePtr untuck_arm_prim = m_mprims.getUntuckArmPrim(true);
        //         MotionPrimitivePtr untuck_arm_prim_part = m_mprims.getUntuckArmPrim(false);
        //         TransitionData t_data;
        //         if (untuck_arm_prim->apply(*source_state, policy_applied_state,t_data)){
        //             edge_prims.clear();
        //             edge_prims.push_back(untuck_arm_prim);
        //             policy_cost = untuck_arm_prim->cost();
        //         } else if (untuck_arm_prim_part->apply(*source_state, policy_applied_state,t_data)) {
        //             edge_prims.clear();
        //             edge_prims.push_back(untuck_arm_prim_part);
        //             policy_cost = untuck_arm_prim_part->cost();
        //         } else {
        //             policy_applied_state.reset(new GraphState(*source_state));
        //         }
        //     } else {
        //         policy_applied_state.reset(new GraphState(*source_state));
        //         // policy here is to find some mprim that has a lower heuristic
        //         // than the current state. We break when we find just one that
        //         // has a lower heuristic.
        //         // int current_arm_heur = m_heur_mgr->getGoalHeuristic(source_state, "admissible_endeff");
        //         std::vector<MotionPrimitivePtr> arm_mprims = m_mprims.getArmMotionPrims();
        //         for (auto& prim : arm_mprims) {
        //             GraphStatePtr arm_policy_applied;
        //             arm_policy_applied.reset(new GraphState(*source_state));
        //             arm_policy_applied->lazyApplyMPrim(prim->getEndCoord());
        //             ContObjectState obj_state_rel_map = arm_policy_applied->getObjectStateRelMapFromState();
        //             ContObjectState current_obj_state = source_state->getObjectStateRelMapFromState();
        //             if (ContObjectState::distance(obj_state_rel_map, m_goal->getObjectState())
        //                  < ContObjectState::distance(current_obj_state, m_goal->getObjectState()))
        //             {
        //                 ROS_DEBUG_NAMED(SEARCH_LOG, "Applying a straight-line arm policy.");
        //                 // bam! we have a policy that reduces the distance
        //                 TransitionData t_data;
        //                 if (!prim->apply(*source_state, policy_applied_state, t_data))
        //                     continue;
        //                 // policy_applied_state = arm_policy_applied;
        //                 edge_prims.clear();
        //                 edge_prims.push_back(prim);
        //                 policy_cost = prim->cost();
        //                 if (heur == 0) {
        //                     ROS_DEBUG_NAMED(SEARCH_LOG, "setting only policy");
        //                     // std::cin.get();
        //                     apply_only_policy = true;
        //                 }
        //                 break;
        //             }
        //         }
        //     }
        // }
    } else {
        policy_applied_state.reset(new GraphState(*source_state));
    }
    for (auto mprim : current_mprims){
        //ROS_DEBUG_NAMED(SEARCH_LOG, "Applying motion:");
        //mprim->printEndCoord();
        GraphStatePtr successor;
        TransitionData t_data;

        if (mprim->motion_type() == MPrim_Types::ARM){
            successor.reset(new GraphState(*policy_applied_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());
            //ROS_INFO("source/successor");
            //mprim->printEndCoord();
            //policy_applied_state->printToInfo(MPRIM_LOG);
            //successor->printToInfo(MPRIM_LOG);
            //ROS_INFO("done");
        } else {
            if (apply_only_policy) {
                ROS_DEBUG_NAMED(SEARCH_LOG, "Whoa. Only policy?");
                // std::cin.get();
                successor.reset(new GraphState(*policy_applied_state));
            } else {
                if (!mprim->apply(*policy_applied_state, successor, t_data)){
                    //ROS_DEBUG_NAMED(MPRIM_LOG, "couldn't apply mprim");
                    continue;
                }
            }
            // ROS_DEBUG_NAMED(SEARCH_LOG, "non-arm mprim/source/successor");
            // mprim->printEndCoord();
            // source_state->printToDebug(MPRIM_LOG);
            // successor->printToDebug(MPRIM_LOG);
            // ROS_DEBUG_NAMED(SEARCH_LOG, "done");
        }

        m_hash_mgr->save(successor);
        Edge key;

        if (m_goal->isSatisfiedBy(successor)){
          m_goal->storeAsSolnState(successor);
          //ROS_DEBUG_NAMED(SEARCH_LOG, "Found potential goal at state %d %d",
          // successor->id(), mprim->cost());
          ROS_INFO("Found potential goal at: source->id %d, successor->id %d,"
            "cost: %d, mprim type: %d ", source_state->id(), successor->id(),
              mprim->cost(), mprim->motion_type());
          succIDs->push_back(GOAL_STATE);
          key = Edge(sourceStateID, GOAL_STATE);
        } else {
          succIDs->push_back(successor->id());
          key = Edge(sourceStateID, successor->id());
        }

        // succIDs->push_back(successor->id());
        // key = Edge(sourceStateID, successor->id());
        std::vector<MotionPrimitivePtr> edge_generator_prims(edge_prims);
        int total_cost = policy_cost;
        if (!apply_only_policy) {
            edge_generator_prims.push_back(mprim);
            total_cost += mprim->cost();
        }
        m_edges.insert(map<Edge, std::vector<MotionPrimitivePtr> >::value_type(key, edge_generator_prims));
        costs->push_back(total_cost);
        isTrueCost->push_back(false);
    }
    // ugly hack because in this particular case, the policy of the base-agent
    // affects the actions of the arm-agent. Thus, we want both sets of actions -
    // base-policy-applied and not.
    if (m_action_partition.at(q_id) == PlanningModes::RIGHT_ARM) {
        ROS_DEBUG_NAMED(SEARCH_LOG, "hacky-policy for the arm queue");
        for (auto mprim : current_mprims){
            GraphStatePtr successor(new GraphState(*source_state));
            TransitionData t_data;

            // successor.reset(new GraphState(*source_state));
            successor->lazyApplyMPrim(mprim->getEndCoord());

            m_hash_mgr->save(successor);
            Edge key;

            if (m_goal->isSatisfiedBy(successor)){
              m_goal->storeAsSolnState(successor);
              ROS_INFO("Found potential goal at: source->id %d, successor->id %d,"
                "cost: %d, mprim type: %d ", source_state->id(), successor->id(),
                  mprim->cost(), mprim->motion_type());
              succIDs->push_back(GOAL_STATE);
              key = Edge(sourceStateID, GOAL_STATE);
            } else {
              succIDs->push_back(successor->id());
              key = Edge(sourceStateID, successor->id());
            }
            std::vector<MotionPrimitivePtr> edge_generator_prims;
            edge_generator_prims.push_back(mprim);
            m_edges.insert(map<Edge, std::vector<MotionPrimitivePtr> >::value_type(key, edge_generator_prims));
            costs->push_back(mprim->cost() + policy_cost);
            isTrueCost->push_back(false);
        }
    }
}

/*
 * Evaluates the edge. Assumes that the edge has already been generated and we
 * know the motion primitive used
 */
int Environment::GetTrueCost(int parentID, int childID){
    TransitionData t_data;

    if (m_edges.find(Edge(parentID, childID)) == m_edges.end()){
      ROS_ERROR("transition hasn't been found between %d and %d??", parentID, childID);
        assert(false);
    }
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);

    ROS_DEBUG_NAMED(SEARCH_LOG, "evaluating edge (%d %d)", parentID, childID);
    GraphStatePtr source_state = m_hash_mgr->getGraphState(parentID);
    GraphStatePtr real_next_successor = m_hash_mgr->getGraphState(childID);
    GraphStatePtr successor, tmp;
    tmp = source_state;
    // MotionPrimitivePtr mprim = m_edges.at(Edge(parentID, childID));
    int total_cost = 0;
    for (auto&& mprim : m_edges.at(Edge(parentID, childID))) {
        if (!mprim->apply(*tmp, successor, t_data)){
            return -1;
        }
        bool valid_successor = (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
                                m_cspace_mgr->isValidTransitionStates(t_data));
        if (!valid_successor){
            return -1;
        }
        total_cost += t_data.cost();
        tmp = successor;
    }
    // mprim->printEndCoord();
    // mprim->print();
    //source_state->printToInfo(SEARCH_LOG);
    //successor->printToInfo(SEARCH_LOG);
    successor->id(m_hash_mgr->getStateID(successor));

    // right before this point, the successor's graph state does not match the
    // stored robot state (because we modified the graph state without calling
    // ik and all that). this call updates the stored robot pose.
    real_next_successor->robot_pose(successor->robot_pose());

    // bool matchesEndID = (successor->id() == childID) || (childID == GOAL_STATE);
    // assert(matchesEndID);

    return total_cost;
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    m_edges.clear();

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    start_pose.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_pose);
    //m_cspace_mgr->visualizeCollisionModel(start_pose);
    //std::cin.get();

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr->save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr->getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    // start_pose.visualize();


    m_goal = search_request->createGoalState();

    if (m_hash_mgr->size() < 2){
        goal_id = saveFakeGoalState(start_graph_state);
    } else {
        goal_id = 1;
    }

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // This informs the adaptive motions about the goal.
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    // informs the heuristic about the goal
    m_heur_mgr->setGoal(*m_goal);

    return true;
}

// a hack to reserve a goal id in the hash so that no real graph state is ever
// saved as the goal state id
int Environment::saveFakeGoalState(const GraphStatePtr& start_graph_state){
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    RobotState fake_robot_state = fake_goal->robot_pose();
    DiscBaseState fake_base = fake_robot_state.base_state();
    fake_base.x(0); fake_base.y(0); fake_base.z(0);
    fake_robot_state.base_state(fake_base);
    fake_goal->robot_pose(fake_robot_state);
    m_hash_mgr->save(fake_goal);
    int goal_id = fake_goal->id();
    assert(goal_id == GOAL_STATE);
    return goal_id;
}

// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                        m_param_catalog.m_robot_resolution_params);
    

    // used for discretization of robot movements
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

#ifdef USE_IKFAST_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using IKFast");
#endif
#ifdef USE_KDL_SOLVER
    ROS_DEBUG_NAMED(CONFIG_LOG, "Using KDL");
#endif

    // Initialize the heuristics. The (optional) parameter defines the cost multiplier.

    m_heur_mgr->initializeHeuristics();

    // used for arm kinematics
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(r_arm.getArmModel(),
                                                  l_arm.getArmModel());
    m_heur_mgr->setCollisionSpaceMgr(m_cspace_mgr);
    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));

    m_action_partition[0] = PlanningModes::RIGHT_ARM_MOBILE;
    m_action_partition[1] = PlanningModes::BASE_ONLY;
    m_action_partition[2] = PlanningModes::BASE_ONLY;
    m_action_partition[3] = PlanningModes::BASE_ONLY;
    m_action_partition[4] = PlanningModes::RIGHT_ARM;
    ROS_DEBUG_NAMED(CONFIG_LOG, "Action Partition map");
    for (auto& ap : m_action_partition) {
        ROS_DEBUG_NAMED(CONFIG_LOG, "%d queue : planning mode : %d", ap.first,
            static_cast<int>(ap.second));
    }
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Setting planning mode to : %d",
        search_request->m_params->planning_mode);
    RobotState::setPlanningMode(search_request->m_params->planning_mode);
}

/*! \brief Given the solution path containing state IDs, reconstruct the
 * actual corresponding robot states. This also makes the path smooth in between
 * each state id because we add in the intermediate states given by the
 * transition data.
 */
vector<FullBodyState> Environment::reconstructPath(vector<int> soln_path){
    PathPostProcessor postprocessor(m_hash_mgr, m_cspace_mgr);
    // std::vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, *m_goal, m_mprims.getMotionPrims());
    std::vector<FullBodyState> final_path = postprocessor.reconstructPath(soln_path, *m_goal,
        m_edges);
    if(m_param_catalog.m_visualization_params.final_path){
        postprocessor.visualizeFinalPath(final_path);
    }
    return final_path;
}

void Environment::generateStartState(SearchRequestPtr search_request) {
    ContObjectState start_obj_state(search_request->m_params->obj_start);
    ContBaseState base_start(search_request->m_params->base_start);
    RobotState start_robot_state(base_start, start_obj_state);
    start_robot_state.visualize();
    m_cspace_mgr->visualizeAttachedObject(start_robot_state);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Generate start state : Keyboard");
    // std::cin.get();
    search_request->m_params->base_start = start_robot_state.getContBaseState();
    search_request->m_params->right_arm_start = start_robot_state.right_arm();
    search_request->m_params->left_arm_start = start_robot_state.left_arm();
}

void Environment::setUseNewHeuristics(bool use_new_heuristics)
{
    m_use_new_heuristics = use_new_heuristics;
    m_heur_mgr->setUseNewHeuristics(m_use_new_heuristics);
}
