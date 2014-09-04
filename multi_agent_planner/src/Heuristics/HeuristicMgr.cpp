#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/StateReps/GoalState.h>
#include <multi_agent_planner/Heuristics/HeuristicMgr.h>
#include <multi_agent_planner/Heuristics/BFS2DHeuristic.h>
#include <costmap_2d/cost_values.h>
#include <memory>
#include <vector>
#include <cstdlib>
#include <cmath>

using namespace multi_agent_planner;

typedef std::pair<int, int> Point;

HeuristicMgr::HeuristicMgr() : 
    m_num_leaders(NUM_LEADERS),
    m_leader_ids(LEADER_IDS)
{ }

HeuristicMgr::~HeuristicMgr()
{
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);

    for (int i=0; i < dimX + 1; i++){
        delete[] m_grid[i];
    }
    delete[] m_grid;
}

/**
 * @brief Resets the heuristic manager.
 */
void HeuristicMgr::reset() {
    ROS_INFO_NAMED(HEUR_LOG, "Resetting the heuristic manager.");
    for (auto& heur: m_heuristic_map) {
        ROS_INFO_NAMED(HEUR_LOG, "shared_ptr count of %s is %d", heur.first.
            c_str(), static_cast<int>(m_heuristics[heur.second].use_count()));
    }
    m_heuristics.clear();
    m_heuristic_map.clear();
    initializeHeuristics();
    // update3DHeuristicMaps();
    update2DHeuristicMaps(m_grid_data);
}

/**
 * @brief sets the planner type - mainly for experiments for the MHA paper
 * @details change the internal planner type to any of the different planners
 */
/*void HeuristicMgr::setPlannerType(int planner_type)
{
    m_planner_type = planner_type;
    switch (planner_type) {
        case T_SMHA:
        case T_IMHA:
        case T_MHG_REEX:
        case T_MHG_NO_REEX:
            m_num_mha_heuristics = 1;
            break;
        case T_ARA:
        case T_MPWA:
            m_num_mha_heuristics = 0;
            break;
        case T_EES:
            m_num_mha_heuristics = 1;
            addUniformCost3DHeur("uniform_3d");
            addUniformCost2DHeur("uniform_2d", 0.7);
            m_heuristics[m_heuristic_map["uniform_2d"]]->update2DHeuristicMap(m_grid_data);
            break;
    }
}
*/
void HeuristicMgr::initializeHeuristics() {
    // initialize as many heuristics as there are number of leaders
    for (int i = 0; i < m_num_leaders; ++i) {
        // the heuristics are named bfs2d_<id> where <id> is the id of the
        // leader.
        std::stringstream ss;
        ss << "bfs2d_" << m_leader_ids[i];
        int cost_multiplier = 1;
        add2DHeur(ss.str(), cost_multiplier);
    }
}

void HeuristicMgr::add2DHeur(std::string name, const int cost_multiplier){
    // Initialize the new heuristic
    BFS2DHeuristicPtr new_2d_heur = std::make_shared<BFS2DHeuristic>();
    // Set cost multiplier here.
    new_2d_heur->setCostMultiplier(cost_multiplier);
    // Add to the list of heuristics
    m_heuristics.push_back(new_2d_heur);
    m_heuristic_map[name] = static_cast<int>(m_heuristics.size() - 1);
}

// most heuristics won't need both 2d and 3d maps. however, the abstract
// heuristic type has function stubs for both of them so we don't need to pick
// and choose who to update. it is up to the implementor to implement a derived
// function for the following, otherwise they won't do anything.
void HeuristicMgr::update2DHeuristicMaps(const std::vector<unsigned char>& data){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);

    m_grid = new unsigned char*[dimX + 1];
    for (int i=0; i < dimX + 1; i++){
        m_grid[i] = new unsigned char[dimY + 1];
        for (int j=0; j < dimY + 1; j++){
            m_grid[i][j] = (data[j*(dimX + 1)+i]);
        }
    }
    m_grid_data.assign(data.begin(), data.end());

    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update2DHeuristicMap(data);
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "Size of m_heuristics: %ld", m_heuristics.size());
}

/**
 * @brief Updates the 3D Heuristic map for heuristics
 */
void HeuristicMgr::update3DHeuristicMaps(){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update3DHeuristicMap();
    }
}

void HeuristicMgr::setGoal(GoalState& goal_state){
    // Save goal state for future use
    m_goal = goal_state;
    auto robots_list = goal_state.getSwarmState().robots_pose();
    // set the right goal for each of the heuristics for the leaders
    for (int i = 0; i < m_num_leaders; i++) {
        std::stringstream ss;
        ss << "bfs2d_" << m_leader_ids[i];
        DiscRobotState d_state = robots_list[m_leader_ids[i]].getDiscRobotState();
        m_heuristics[m_heuristic_map[ss.str()]]->setGoal(d_state.x(), d_state.y());
        ROS_DEBUG_NAMED(HEUR_LOG, "[HEUR_LOG] setting goal %d %d for leader_id %d",
                                    d_state.x(), d_state.y(), m_leader_ids[i]);
    }
}

void HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state,
                                        std::unique_ptr<stringintmap>& values)
{
    if (!m_heuristics.size()){
        ROS_ERROR_NAMED(HEUR_LOG, "No heuristics initialized!");
        return;
    }
    values.reset(new stringintmap(m_heuristic_map));
    for (int i = 0; i < m_num_leaders; i++) {
        std::stringstream ss;
        ss << "bfs2d_" << m_leader_ids[i];
        values->at(ss.str()) = 
                        m_heuristics[m_heuristic_map.at(ss.str())]->
                                    getGoalHeuristic(state, m_leader_ids[i]);
    }
}

int HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state, std::string name,
    int leader_id)
{
    assert(!m_heuristics.empty());
    return m_heuristics[m_heuristic_map[name]]->getGoalHeuristic(state, leader_id);
}

void HeuristicMgr::printSummaryToInfo(char* logger){
    ROS_INFO_NAMED(logger, "--------------------------");
    ROS_INFO_NAMED(logger, "Summary of heuristics");
    ROS_INFO_NAMED(logger, "--------------------------");

    ROS_INFO_NAMED(logger, "Size of m_heuristics: %d", static_cast<int>(
        m_heuristic_map.size()));
    ROS_INFO_NAMED(logger, "What they are : ");
    for (auto& heuristic: m_heuristic_map){
        ROS_INFO_NAMED(logger, "%s -- id %d",
            heuristic.first.c_str(), heuristic.second);
    }
}

void HeuristicMgr::printSummaryToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "--------------------------");
    ROS_DEBUG_NAMED(logger, "Summary of heuristics");
    ROS_DEBUG_NAMED(logger, "--------------------------");

    ROS_DEBUG_NAMED(logger, "Size of m_heuristics: %d", static_cast<int>(
        m_heuristic_map.size()));
    ROS_DEBUG_NAMED(logger, "What they are : ");
    for (auto& heuristic: m_heuristic_map){
        ROS_DEBUG_NAMED(logger, "%s -- id %d",
            heuristic.first.c_str(), heuristic.second);
    }
}
