#include <ros/ros.h>
#include <multi_agent_planner_node/GetSwarmPlan.h>
#include <multi_agent_planner_node/StatsWriter.h>
#include <multi_agent_planner/Constants.h>
#include <sbpl/planners/mha_planner.h>

using namespace multi_agent_planner;

void printUsage(){
  printf("usage: runTests path_to_test_file.yaml [trial_id] [test_num]\n");
}


int main(int argc, char** argv){
  if(argc < 2 || argc > 4){
    printUsage();
    return 1;
  }
  ros::init(argc,argv,"run_tests");
  ros::NodeHandle nh;

  multi_agent_planner_node::GetSwarmPlan::Request req;
  multi_agent_planner_node::GetSwarmPlan::Response res;

  char* filename;
  filename = argv[1];

  int trial_id = 42;
  if (argc >= 3)
    trial_id = std::atoi(argv[2]);

  int test_num_to_run = -1;
  if (argc == 4)
    test_num_to_run = std::atoi(argv[3]);

  //planner parameters
  req.initial_eps = 2.0;
  req.final_eps = 2.0;
  req.dec_eps = 0.2;

  std::vector<double> start_swarm(2,0);
  std::vector<double> goal_swarm(2,0);

  req.swarm_start = start_swarm;
  req.swarm_goal = goal_swarm;
  req.initial_eps = 2.0;
  req.tolerance = .02;

  req.allocated_planning_time = 30;

  req.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
  req.planner_type = mha_planner::PlannerType::SMHA;

  ros::service::waitForService("/sbpl_planning/plan_path",10);
  ros::ServiceClient planner = ros::NodeHandle().serviceClient<multi_agent_planner_node::GetSwarmPlan>("/sbpl_planning/plan_path", true);
  sleep(1);

  FILE* fin = fopen(filename,"r");
  if(!fin){
    printf("file %s does not exist\n", argv[1]);
    return 1;
  }
  fscanf(fin,"experiments:\n\n");

  bool first = true;
  while(1){
    int test_num = 0;
    if(fscanf(fin,"  - test: test_%d\n    start:\n", &test_num) <= 0)
      break;

    if(fscanf(fin,"      position: %lf %lf\n    goal:\n",
              &req.swarm_start[RobotStateElement::X],
              &req.swarm_start[RobotStateElement::Y]) <= 0)
      break;
    if(fscanf(fin,"      position: %lf %lf\n",
              &req.swarm_goal[RobotStateElement::X],
              &req.swarm_goal[RobotStateElement::Y]) <= 0)
      break;

    if (test_num != test_num_to_run && test_num_to_run!=-1){
      printf("(skipping %d)\n", test_num);
      continue;
    }
    printf("Running test %d\n",test_num);
    bool isPlanFound = planner.call(req,res);
    if (isPlanFound) {
      std::stringstream stats_file_name;
      stats_file_name << "swarm_stats_" << trial_id << ".csv";
      StatsWriter::writeStatsToFile(stats_file_name.str().c_str(), first, res.stats_field_names, res.stats);
      first = false;
    }
  }

  return 0;
}
