#include <multi_agent_planner_node/GetSwarmPlan.h>
#include <multi_agent_planner/Constants.h>
#include <sbpl/planners/mha_planner.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ros/ros.h>
#include <vector>

int main(int argc, char** argv){
    ros::init(argc, argv, "testPlanningRequest");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<multi_agent_planner_node::GetSwarmPlan>("/sbpl_planning/plan_path");
    multi_agent_planner_node::GetSwarmPlan srv;

    // std::vector<double> start_swarm { 0.6, 0.3 };
    // std::vector<double> goal_swarm { 6, 2.5};
    
    std::vector<double> start_swarm { 0.6, 0.3 };
    std::vector<double> goal_swarm { 0.6, 5.5};

    srv.request.swarm_start = start_swarm;
    srv.request.swarm_goal = goal_swarm;
    srv.request.initial_eps = 2.0;
    srv.request.final_eps = 2.0;
    srv.request.dec_eps = .1;
    srv.request.tolerance = .02;

    srv.request.allocated_planning_time = 30;

    srv.request.meta_search_type = mha_planner::MetaSearchType::ROUND_ROBIN;
    srv.request.planner_type = mha_planner::PlannerType::SMHA;

    ROS_INFO("Sending request at : %s",
        boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time()).c_str());
    if (client.call(srv))
    {
        ROS_INFO("called service");
        for (size_t i=0; i < srv.response.stats_field_names.size(); i++){
            ROS_INFO("%s: %f", srv.response.stats_field_names[i].c_str(),
                               srv.response.stats[i]);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    return 0;
}
