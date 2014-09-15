#include <multi_agent_planner/Heuristics/BFS2DHeuristic.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Visualizer.h>
#include <sbpl/utils/key.h>
#include <stdexcept>
#include <costmap_2d/cost_values.h>

using namespace multi_agent_planner;

BFS2DHeuristic::BFS2DHeuristic(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX+1);
    m_size_row = static_cast<unsigned int>(dimY+1);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
                              m_size_col, m_size_row);

    m_gridsearch.reset(new SBPL2DGridSearch(m_size_col, m_size_row,
        m_occupancy_grid->getResolution()));
    
    // Initialize the grid here itself so that you don't wait for the map
    // callback to be called
    m_grid = new unsigned char*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new unsigned char[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = 0;
        }
    }

    // m_circlepub = m_nh.advertise<geometry_msgs::PolygonStamped>("/goal_circle", 1,
        // true);
}

BFS2DHeuristic::~BFS2DHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete[] m_grid[i];
    }
    delete[] m_grid;
    m_grid = NULL;
}

void BFS2DHeuristic::update2DHeuristicMap(const std::vector<unsigned char>& data){
    loadMap(data);
}

void BFS2DHeuristic::loadMap(const std::vector<unsigned char>& data){

    for (unsigned int j=0; j < m_size_row; j++){
        for (unsigned int i=0; i < m_size_col; i++){
            m_grid[i][j] = (data[j*m_size_col+i]);
        }
    }

    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map", m_size_col, m_size_row);
}

void BFS2DHeuristic::setGoal(int x, int y) {
    std::vector<std::pair<int, int> > init_points { {x, y} };
    m_gridsearch->search(m_grid, costmap_2d::INSCRIBED_INFLATED_OBSTACLE, x, y,
        0,0, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS, init_points);
}

void BFS2DHeuristic::setGoal(GoalState& goal_state){
    throw std::runtime_error("You shouldn't be calling this!");
}

int BFS2DHeuristic::getGoalHeuristic(GraphStatePtr state, int leader_id) {
    // get the leader's coordinates
    DiscRobotState d_robot_state = 
            state->swarm_state().robots_pose()[leader_id].getDiscRobotState();
    int x = d_robot_state.x();
    int y = d_robot_state.y();
    
    // Check if within bounds. We need to do this here because the bfs2d
    // implementation doesn't take care of this.
    if (x < 0 || x >= int(m_size_col) || y < 0 || y >= int(m_size_row)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d", x, y);
        return INFINITECOST;
    }

    int cost = m_gridsearch->getlowerboundoncostfromstart_inmm(x, y);
    // ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] 2Ddijkstra's cost to %d %d is %d, mult %d", 
    //                           x, y, cost, getCostMultiplier());
    
    if (cost < 0){
        return INFINITECOST;
    }

    return getCostMultiplier()*cost;
}

/**
 * @brief visualizes the path from (x,y) to the goal
 */
void BFS2DHeuristic::visualizePath(int x, int y) {
    // get the path
    std::vector<std::pair<int,int> > path;
    m_gridsearch->getPath(x, y, path);
    // convert to geometry_msgs::Point
    std::vector<geometry_msgs::Point> points;
    for (auto& path_point : path) {
        geometry_msgs::Point pt;
        pt.x = path_point.first;
        pt.y = path_point.second;
        pt.z = 0;
        points.push_back(pt);
    }
    Visualizer::swarmVizPtr->visualizeLine(points, "heur_path", 42, 0, 0.1);
}

// void BFS2DHeuristic::visualizeCenter(int x, int y) {
//     ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2DHeuristic] Visualizing %d %d,", x, y);
//     std::vector <std::vector <double> > startpoint;
//     std::vector <double> color(4, 1);
//     color[1] = color[2] = 0;
//     std::vector<double> point;
//     point.push_back(x*m_occupancy_grid->getResolution());
//     point.push_back(y*m_occupancy_grid->getResolution());
//     point.push_back(0.0);
//     startpoint.push_back(point);
//     std::stringstream ss;
//     ss << "start_point" << x << "_" << y;
//     Visualizer::pviz->visualizeBasicStates(startpoint, color, ss.str(), 0.02);
// }
