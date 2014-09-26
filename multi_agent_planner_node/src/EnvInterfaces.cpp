#include <multi_agent_planner_node/EnvInterfaces.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/Constants.h>
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
extern int multi_agent_planner::SwarmState::MIDDLE_GUY;
extern std::vector<std::vector<multi_agent_planner::ContRobotState> > multi_agent_planner::SwarmState::REL_POSITIONS;

// constructor automatically launches the collision space interface, which only
// loads it up with a pointer to the collision space mgr. it doesn't bind to any
// topic.
EnvInterfaces::EnvInterfaces(std::shared_ptr<multi_agent_planner::Environment> env, ros::NodeHandle nh) :
    m_nodehandle(nh),
    m_env(env), m_collision_space_interface(new CollisionSpaceInterface(env->getCollisionSpace(), env->getHeuristicMgr())),
    m_navmap_handler(new NavMapHandler(m_nodehandle))
{
    m_collision_space_interface->mutex = &mutex;
    getParams();
    bool forward_search = true;
    m_ara_planner.reset(new LazyARAPlanner(m_env.get(), forward_search));
    m_mha_planner.reset(new MHAPlanner(m_env.get(), NUM_LEADERS+1, forward_search));

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
        bool valid_inflation = true;
        for (size_t i = 0; i < swarm_start.robots_pose().size(); ++i) {
            auto disc_robot = swarm_start.robots_pose()[i].getDiscRobotState();
            if (m_grid[disc_robot.x()][disc_robot.y()] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                valid_inflation = false;
                break;
            }
        }
        for (size_t i = 0; i < swarm_goal.robots_pose().size() &&
            valid_inflation; ++i) {
            auto disc_robot = swarm_goal.robots_pose()[i].getDiscRobotState();
            if (m_grid[disc_robot.x()][disc_robot.y()] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
                valid_inflation = false;
                break;
            }
        }
        if (!valid_inflation)
            continue;
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
    GetSwarmPlan::Request &req,
    GetSwarmPlan::Response &res,
    SearchRequestParamsPtr search_request) {

    int start_id, goal_id;
    bool forward_search = true;
    clock_t total_planning_time;
    bool isPlanFound;
    std::vector<double> stats;
    std::vector<std::string> stat_names;
    std::vector<SwarmState> states;

    int planner_queues;
    planner_queues = static_cast<int>(SwarmState::LEADER_IDS.size())
                    + 1 /*anchor */
                    + 1 /*swarm_inscribed*/;
    ROS_INFO("Creating planner with %d queues", planner_queues);
    m_env->reset();
    // set the planner type
    m_env->setPlannerType(static_cast<multi_agent_planner::PlannerType::Type>(req.sbpl_planner));
    m_mha_planner.reset(new MHAPlanner(m_env.get(), planner_queues, forward_search));
    total_planning_time = clock();
    if (!m_env->configureRequest(search_request, start_id, goal_id)){
        ROS_ERROR("Unable to configure request for mha_planner!");
        return false;
    }
    m_mha_planner->set_start(start_id);
    ROS_INFO("setting mha_planner goal id to %d", goal_id);
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
        ROS_INFO("Plan found in mha_planner. Moving on to reconstruction.");
        states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        bool mha_planner = true;
        packageStats(stat_names, stats, states.size(), mha_planner);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        // packageStats(stat_names, stats, soln_cost, states.size(),
            // total_planning_time);
        ROS_INFO("No plan found in mha_planner!");
    }
    return isPlanFound;
}

bool EnvInterfaces::runARAPlanner(
    GetSwarmPlan::Request &req,
    GetSwarmPlan::Response &res,
    SearchRequestParamsPtr search_request) {

    int start_id, goal_id;
    bool forward_search = true;
    clock_t total_planning_time;
    bool isPlanFound;
    std::vector<double> stats;
    std::vector<std::string> stat_names;
    std::vector<SwarmState> states;

    m_env->reset();
    // set the planner type
    m_env->setPlannerType(static_cast<multi_agent_planner::PlannerType::Type>(req.sbpl_planner));
    m_ara_planner.reset(new LazyARAPlanner(m_env.get(), forward_search));
    total_planning_time = clock();
    if (!m_env->configureRequest(search_request, start_id, goal_id)){
        ROS_ERROR("Unable to configure request for LazyARAPlanner!");
        return false;
    }
    m_ara_planner->set_start(start_id);
    ROS_INFO("setting LazyARAPlanner goal id to %d", goal_id);
    m_ara_planner->set_goal(goal_id);
    m_ara_planner->force_planning_from_scratch();
    
    std::vector<int> soln;
    int soln_cost;
    ReplanParams replan_params(req.allocated_planning_time);
    replan_params.initial_eps = EPS1*EPS2;
    replan_params.final_eps = EPS1*EPS2;
    replan_params.return_first_solution = false;

    isPlanFound = m_ara_planner->replan(&soln, replan_params, &soln_cost);

    if (isPlanFound) {
        ROS_INFO("Plan found in LazyARAPlanner. Moving on to reconstruction.");
        states =  m_env->reconstructPath(soln);
        total_planning_time = clock() - total_planning_time;
        bool mha_planner = false;
        packageStats(stat_names, stats, states.size(), mha_planner);
        res.stats_field_names = stat_names;
        res.stats = stats;
    } else {
        // bool mha_planner = false;
        // packageStats(stat_names, stats, soln_cost, states.size(),
        //     total_planning_time, mha_planner);
        ROS_INFO("No plan found in LazyARAPlanner!");
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
    bool isPlanFound;

    auto sbpl_planner = static_cast<multi_agent_planner::PlannerType::Type>(req.sbpl_planner);
    if (sbpl_planner == multi_agent_planner::PlannerType::Type::MHA)
        isPlanFound = runMHAPlanner(req, res, search_request);
    else
        isPlanFound = runARAPlanner(req, res, search_request);
    return isPlanFound;
}

void EnvInterfaces::packageStats(std::vector<std::string>& stat_names,
                                 std::vector<double>& stats,
                                 size_t solution_size,
                                 bool mha_planner = true)
{
    stat_names.resize(12);
    stats.resize(12);
    stat_names[0] = "total_plan_time";
    stat_names[1] = "initial_solution_planning_time";
    stat_names[2] = "epsilon_1";
    stat_names[3] = "initial_solution_expansions";
    stat_names[4] = "final_epsilon_planning_time";
    stat_names[5] = "epsilon_2";
    stat_names[6] = "solution_epsilon";
    stat_names[7] = "expansions";
    stat_names[8] = "solution_cost";
    stat_names[9] = "path_length";
    stat_names[10] = "num_leader_changes";
    stat_names[11] = "num_gen_successors";

    std::vector<PlannerStats> planner_stats;
    if (mha_planner)
        m_mha_planner->get_search_stats(&planner_stats);
    else
        m_ara_planner->get_search_stats(&planner_stats);
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
    stats[11] = m_env->getNumGeneratedSuccessors();
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


void EnvInterfaces::loadNavMap(const nav_msgs::OccupancyGridPtr& map){
    boost::unique_lock<boost::mutex> lock(mutex);
    ROS_DEBUG_NAMED(CONFIG_LOG, "received navmap of size %u %u, resolution %f",
                    map->info.width, map->info.height, map->info.resolution);
    ROS_DEBUG_NAMED(CONFIG_LOG, "origin is at %f %f", map->info.origin.position.x,
                                                      map->info.origin.position.y);
    m_navmap_handler->loadNavMap(map);

    // look up the values from the occup grid parameters
    // This stuff is in cells.
    int dimX, dimY, dimZ;
    m_collision_space_interface->getOccupancyGridSize(dimX, dimY,
        dimZ);
    ROS_DEBUG_NAMED(CONFIG_LOG, "Size of OccupancyGrid : %d %d %d", dimX, dimY,
        dimZ);
    m_navmap_handler->setOccupancyDims(dimX, dimY);

    m_navmap_handler->addCostmap("costmap_2d");
    m_navmap_handler->addCostmap("costmap_2d_swarm_inscribed");

    // get the bigger inflation
    std::vector<unsigned char> inscribed_inflation;
    m_navmap_handler->getInflatedMap("costmap_2d_swarm_inscribed", inscribed_inflation);
    m_env->getHeuristicMgr()->updateAdditional2DMaps(inscribed_inflation);
    

    // get the smaller inflation
    std::vector<unsigned char> cropped_map;
    m_navmap_handler->getInflatedMap("costmap_2d", cropped_map);

    m_env->getPolicyGenerator()->update2DHeuristicMaps(cropped_map);
    m_env->getHeuristicMgr()->update2DHeuristicMaps(cropped_map);

    // save as grid for use when generating random start-goals
    m_grid = new unsigned char*[dimX + 1];
    for (int i=0; i < dimX + 1; i++){
        m_grid[i] = new unsigned char[dimY + 1];
        for (int j=0; j < dimY + 1; j++){
            m_grid[i][j] = (cropped_map[j*(dimX + 1)+i]);
        }
    }
}