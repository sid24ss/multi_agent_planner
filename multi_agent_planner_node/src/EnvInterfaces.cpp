#include <multi_agent_planner_node/EnvInterfaces.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <boost/filesystem.hpp>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>
#include <climits>
#include <string>

using namespace multi_agent_planner_node;
using namespace multi_agent_planner;
using namespace boost;

// ugly as hell.
extern std::vector<int> multi_agent_planner::SwarmState::LEADER_IDS;
extern int multi_agent_planner::SwarmState::NUM_ROBOTS;
extern std::vector<std::vector<multi_agent_planner::ContRobotState> > multi_agent_planner::SwarmState::REL_POSITIONS;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.
EnvInterfaces::EnvInterfaces(std::shared_ptr<multi_agent_planner::Environment> env, ros::NodeHandle nh) :
    m_nodehandle(nh),
    m_env(env), m_collision_space_interface(new CollisionSpaceInterface(env->getCollisionSpace(), env->getHeuristicMgr())),
    // This costmap_ros object listens to the map topic as defined
    // in the costmap_2d.yaml file.
    m_costmap_ros(new costmap_2d::Costmap2DROS("costmap_2d", m_tf))

{
    m_collision_space_interface->mutex = &mutex;
    getParams();
    bool forward_search = true;
    m_ara_planner.reset(new ARAPlanner(m_env.get(), forward_search));
    m_mha_planner.reset(new MHAPlanner(m_env.get(), NUM_LEADERS+1, forward_search));
    m_costmap_pub = m_nodehandle.advertise<nav_msgs::OccupancyGrid>("costmap_pub", 1);
    m_costmap_publisher.reset(new costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));

    interrupt_sub_ = nh.subscribe("/sbpl_planning/interrupt", 1, &EnvInterfaces::interruptPlannerCallback,this);
}

void EnvInterfaces::interruptPlannerCallback(std_msgs::EmptyConstPtr){
  ROS_WARN("Planner interrupt received!");
  m_mha_planner->interrupt();
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<std::string>("reference_frame", m_params.ref_frame, 
                                    std::string("map"));
}

void EnvInterfaces::bindPlanPathToEnv(std::string service_name){
    m_plan_service = m_nodehandle.advertiseService(service_name, 
                                                   &EnvInterfaces::planPathCallback,
                                                   this);
}

void EnvInterfaces::bindWriteExperimentToEnv(std::string service_name){
    m_write_exp_service = m_nodehandle.advertiseService(service_name, 
                                   &EnvInterfaces::GenerateExperimentsFile,
                                   this);
}

bool EnvInterfaces::GenerateExperimentsFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("generating trials!");
  std::vector<std::pair<ContPoint, ContPoint> > start_goal_pairs;
  int number_of_trials = 10;
  int dimX, dimY, dimZ;
  m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
  double maxX = dimX * ContRobotState::getResolution();
  double maxY = dimY * ContRobotState::getResolution();
  // generate random states and check validity; random states (at the moment)
  // are just x,y for the reference point on the swarm.
  int successful_pair_count = 0;
  while (successful_pair_count < number_of_trials) {
    std::vector<double> start {randomDouble(0, maxX), randomDouble(0,maxY)};
    std::vector<double> goal {randomDouble(0, maxX), randomDouble(0,maxY)};
    auto swarm_start = SwarmState::transformSwarmToPos(start);
    auto swarm_goal = SwarmState::transformSwarmToPos(goal);
    if (m_collision_space_interface->getCollisionSpace()->isValid(swarm_start)
        && m_collision_space_interface->getCollisionSpace()->isValid(swarm_goal))
    {
        swarm_start.visualize();
        swarm_goal.visualize();
        // huzzah! We have a valid start goal
        ContPoint start_pt(std::make_pair(start[RobotStateElement::X],
                                        start[RobotStateElement::Y]));
        ContPoint goal_pt(std::make_pair(goal[RobotStateElement::X],
                                        goal[RobotStateElement::Y]));
        start_goal_pairs.push_back(std::make_pair(start_pt, goal_pt));
        ROS_INFO("generated pair %d", successful_pair_count);
        successful_pair_count++;
    }
  }
  std::string out_path;
  m_nodehandle.param<std::string>("experiments/out_path", out_path, 
    std::string("/home/siddharth/ros-packages/multiagent/multi_agent_planner_node/experiments"));
  out_path.append("/swarm_tests.yaml");
  int test_num = 0;
  ROS_INFO("writing to file %s", out_path.c_str());
  FILE* fout = fopen(out_path.c_str(),"w");
  fprintf(fout, "experiments:\n\n");
  for (auto& start_goal : start_goal_pairs){
    fprintf(fout,"  - test: test_%d\n", test_num);
    fprintf(fout,"    start:\n");
    fprintf(fout,"      position: %f %f\n",
        start_goal.first.first,
        start_goal.first.second);
    fprintf(fout,"    goal:\n");
    fprintf(fout,"      position: %f %f\n",
        start_goal.second.first,
        start_goal.second.second);
    fprintf(fout,"\n");
    test_num++;
  }
  fclose(fout);
  return true;
}

bool EnvInterfaces::runMHAPlanner(
    std::string planner_prefix,
    GetSwarmPlan::Request &req,
    GetSwarmPlan::Response &res,
    SearchRequestParamsPtr search_request,
    int counter) {

    int start_id, goal_id;
    bool forward_search = true;
    clock_t total_planning_time;
    bool isPlanFound;
    std::vector<double> stats;
    std::vector<std::string> stat_names;
    std::vector<SwarmState> states;

    int planner_queues;
    planner_queues = static_cast<int>(SwarmState::LEADER_IDS.size()) + 1;
    ROS_INFO("Creating planner with %d queues", planner_queues);
    m_env->reset();
    m_mha_planner.reset(new MHAPlanner(m_env.get(), planner_queues, forward_search));
    total_planning_time = clock();
    if (!m_env->configureRequest(search_request, start_id, goal_id)){
        ROS_ERROR("Unable to configure request for %s! Trial ID: %d",
         planner_prefix.c_str(), counter);
        return false;
    }
    m_mha_planner->set_start(start_id);
    ROS_INFO("setting %s goal id to %d", planner_prefix.c_str(), goal_id);
    m_mha_planner->set_goal(goal_id);
    m_mha_planner->force_planning_from_scratch();
    std::vector<int> soln;
    int soln_cost;
    MHAReplanParams replan_params(req.allocated_planning_time);
    replan_params.inflation_eps = EPS1;
    replan_params.anchor_eps = EPS2;
    replan_params.use_anchor = true;
    replan_params.return_first_solution = true;

    replan_params.meta_search_type = static_cast<mha_planner::MetaSearchType>(req.meta_search_type);
    replan_params.planner_type = static_cast<mha_planner::PlannerType>(req.planner_type);

    isPlanFound = m_mha_planner->replan(&soln, replan_params, &soln_cost);

    if (isPlanFound) {
        ROS_INFO("Plan found in %s Planner. Moving on to reconstruction.",
            planner_prefix.c_str());
        states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        packageMHAStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        packageMHAStats(stat_names, stats, soln_cost, states.size(),
            total_planning_time);
        ROS_INFO("No plan found in %s!", planner_prefix.c_str());
    }
    return isPlanFound;
}

bool EnvInterfaces::planPathCallback(GetSwarmPlan::Request &req, 
                                     GetSwarmPlan::Response &res)
{
    boost::unique_lock<boost::mutex> lock(mutex);

    SearchRequestParamsPtr search_request = std::make_shared<SearchRequestParams>();
    search_request->initial_epsilon = req.initial_eps;
    search_request->final_epsilon = req.final_eps;
    search_request->decrement_epsilon = req.dec_eps;

    search_request->tolerance = req.tolerance;
    
    // configure start swarm
    search_request->swarm_start = req.swarm_start;

    // configure goal swarm
    search_request->swarm_goal = req.swarm_goal;

    res.stats_field_names.resize(18);
    res.stats.resize(18);
    int counter = 42;
    bool isPlanFound;

    isPlanFound = runMHAPlanner("smha_", req, res, search_request, counter);
    return isPlanFound;
}

void EnvInterfaces::packageMHAStats(std::vector<std::string>& stat_names,
                                 std::vector<double>& stats,
                                 int solution_cost,
                                 size_t solution_size,
                                 double total_planning_time)
{
    stat_names.resize(11);
    stats.resize(11);
    stat_names[0] = "total plan time";
    stat_names[1] = "initial solution planning time";
    stat_names[2] = "epsilon 1";
    stat_names[3] = "initial solution expansions";
    stat_names[4] = "final epsilon planning time";
    stat_names[5] = "epsilon 2";
    stat_names[6] = "solution epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution cost";
    stat_names[9] = "path length";
    stat_names[10] = "num_leader_changes";

    // TODO fix the total planning time
    //stats[0] = totalPlanTime;
    // TODO: Venkat. Handle the inital/final solution eps correctly when this becomes anytime someday.
    
    // stats[0] = total_planning_time/static_cast<double>(CLOCKS_PER_SEC);
    // stats[1] = m_ara_planner->get_initial_eps_planning_time();
    // stats[2] = m_ara_planner->get_initial_eps();
    // stats[3] = m_ara_planner->get_n_expands_init_solution();
    // stats[4] = m_ara_planner->get_final_eps_planning_time();
    // stats[5] = m_ara_planner->get_final_epsilon();
    // stats[6] = m_ara_planner->get_solution_eps();
    // stats[7] = m_ara_planner->get_n_expands();
    // stats[8] = static_cast<double>(solution_cost);
    // stats[9] = static_cast<double>(solution_size);
    
    std::vector<PlannerStats> planner_stats;
    m_mha_planner->get_search_stats(&planner_stats);
    // Take stats only for the first solution, since this is not anytime currently
    stats[0] = planner_stats[0].time;
    stats[1] = stats[0];
    stats[2] = EPS1*EPS2;
    stats[3] = planner_stats[0].expands;
    stats[4] = stats[0];
    stats[5] = stats[2];
    stats[6] = stats[2];
    stats[7] = stats[3];
    stats[8] = static_cast<double>(planner_stats[0].cost);
    stats[9] = static_cast<double>(solution_size);
    stats[10] = m_env->getLeaderChangeCount();
}

bool EnvInterfaces::bindCollisionSpaceToTopic(std::string topic_name){
    m_collision_space_interface->bindCollisionSpaceToTopic(topic_name, 
                                                          m_tf, 
                                                          m_params.ref_frame);
    return true;
}

void EnvInterfaces::bindNavMapToTopic(std::string topic){
    // sleep(2.0);//TODO: ??!!*U8084u
    m_nav_map = m_nodehandle.subscribe(topic, 1, &EnvInterfaces::loadNavMap, this);
}

void EnvInterfaces::crop2DMap(const nav_msgs::MapMetaData& map_info, const
    std::vector<unsigned char>& v, double new_origin_x, double new_origin_y,
                              double width, double height)
{
    ROS_DEBUG_NAMED(CONFIG_LOG, "to be cropped to : %f (width), %f (height)", width, height);
    std::vector<std::vector<unsigned char> > tmp_map(map_info.height);
    for (unsigned int i=0; i < map_info.height; i++){
        for (unsigned int j=0; j < map_info.width; j++){
            tmp_map[i].push_back(v[i*map_info.width+j]);
        }
    }

    double res = map_info.resolution;
    ROS_DEBUG_NAMED(CONFIG_LOG, "resolution : %f", res);
    int new_origin_x_idx = (new_origin_x-map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-map_info.origin.position.y)/res;
    int new_width = static_cast<int>((width/res) + 1 + 0.5);
    int new_height = static_cast<int>((height/res) + 1 + 0.5);
    ROS_DEBUG_NAMED(HEUR_LOG, "new origin: %d %d, new_width and new_height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(HEUR_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    std::vector<std::vector<unsigned char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }
    m_final_map.clear();
    m_cropped_map.clear();
    // m_final_map.resize(new_width * new_height);
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++){
            m_final_map.push_back(static_cast<signed char>(double(new_map[i][j])/254.0*100.0));
            m_cropped_map.push_back(new_map[i][j]);
        }
    }
    ROS_DEBUG_NAMED(HEUR_LOG, "size of final map: %lu", m_final_map.size());
}

void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    boost::unique_lock<boost::mutex> lock(mutex);
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Size of OccupancyGrid : %d %d %d", dimX, dimY,
        dimZ);

    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    // Normalize and convert to array.
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            // Row major. X is row wise, Y is column wise.
            int c = cost_map.getCost(i,j);

            // Set unknowns to free space (we're dealing with static maps for
            // now)
            if (c == costmap_2d::NO_INFORMATION) {
                c = costmap_2d::FREE_SPACE;
            }
            // c = (c == (costmap_2d::NO_INFORMATION)) ? (costmap_2d::FREE_SPACE) : (c);

            // Re-set the cost.
            cost_map.setCost(i,j,c);
        }
    }

    // Re-inflate because we modified the unknown cells to be free space.
    // API : center point of window x, center point of window y, size_x ,
    // size_y
    cost_map.reinflateWindow(dimX*map->info.resolution/2, dimY*map->info.resolution/2, dimX*map->info.resolution, dimY*map->info.resolution);

    std::vector<unsigned char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
          uncropped_map.push_back(cost_map.getCost(i,j));
        }
    }

    m_costmap_publisher->updateCostmapData(cost_map, m_costmap_ros->getRobotFootprint());

    // Publish the full costmap
    // topic : /multi_agent_planner_node/inflated_obstacles (RViz: Grid
    // Cells)
    m_costmap_publisher->publishCostmap();
    // topic : /multi_agent_planner_node/robot_footprint (RViz: polygon)
    m_costmap_publisher->publishFootprint();

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*map->info.resolution;
    double height = dimY*map->info.resolution;
    
    crop2DMap(map->info, uncropped_map, 0, 0, width, height);
    
    // Don't want to publish this.
    nav_msgs::OccupancyGrid costmap_pub;
    costmap_pub.header.frame_id = "/map";
    costmap_pub.header.stamp = ros::Time::now();
    costmap_pub.info.map_load_time = ros::Time::now();
    costmap_pub.info.resolution = map->info.resolution;
    // done in the crop function too.
    costmap_pub.info.width = (width/map->info.resolution+1 + 0.5);
    costmap_pub.info.height = (height/map->info.resolution+1 + 0.5);
    costmap_pub.info.origin.position.x = 0;
    costmap_pub.info.origin.position.y = 0;
    costmap_pub.data = m_final_map;

    // Publish the cropped version of the costmap; publishes
    // /multi_agent_planner/costmap_pub
    ROS_INFO_NAMED(CONFIG_LOG, "Publishing the final map that's supposed to fit"
        " within the occupancy grid.");
    m_costmap_pub.publish(costmap_pub);

    m_env->getPolicyGenerator()->update2DHeuristicMaps(m_cropped_map);
    m_env->getHeuristicMgr()->update2DHeuristicMaps(m_cropped_map);
}