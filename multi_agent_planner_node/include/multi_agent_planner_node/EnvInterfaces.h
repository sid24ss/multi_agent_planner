#pragma once
#include <multi_agent_planner_node/CollisionSpaceInterface.h>
#include <multi_agent_planner_node/NavMapHandler.h>
#include <multi_agent_planner_node/GetSwarmPlan.h>
#include <multi_agent_planner/PathPostProcessor.h>
#include <multi_agent_planner/SearchRequest.h>
#include <multi_agent_planner/Environment.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <sbpl/planners/mha_planner.h>
#include <sbpl/planners/lazyARA.h>
#include <sbpl/planners/planner.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <string>

namespace multi_agent_planner_node {
    struct InterfaceParams {
        std::string ref_frame;
    };

    class EnvInterfaces {
        public:
            EnvInterfaces(std::shared_ptr<multi_agent_planner::Environment> env,
                ros::NodeHandle nh);
            void getParams();
            bool planPathCallback(GetSwarmPlan::Request &req, 
                                  GetSwarmPlan::Response &res);
            void bindPlanPathToEnv(std::string service_name);
            void bindWriteExperimentToEnv(std::string service_name);
            bool bindCollisionSpaceToTopic(std::string topic_name);
            void bindNavMapToTopic(std::string topic_name);
            void packageStats(std::vector<std::string>& stat_names,
                              std::vector<double>& stats,
                              size_t solution_size,
                              bool mha_planner);
             bool GenerateExperimentsFile(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        private:
            void loadNavMap(const nav_msgs::OccupancyGridPtr& map);
            void interruptPlannerCallback(std_msgs::EmptyConstPtr);
            bool runMHAPlanner(
                  GetSwarmPlan::Request &req,
                  GetSwarmPlan::Response &res,
                  multi_agent_planner::SearchRequestParamsPtr search_request);
            bool runARAPlanner(
                  GetSwarmPlan::Request &req,
                  GetSwarmPlan::Response &res,
                  multi_agent_planner::SearchRequestParamsPtr search_request);
            
            ros::NodeHandle m_nodehandle;
            InterfaceParams m_params;
            std::shared_ptr<multi_agent_planner::Environment> m_env;
            std::unique_ptr<CollisionSpaceInterface> m_collision_space_interface;
            tf::TransformListener m_tf;

            ros::ServiceServer m_plan_service;
            ros::ServiceServer m_write_exp_service;

            std::unique_ptr<LazyARAPlanner> m_ara_planner;
            std::unique_ptr<MHAPlanner> m_mha_planner;

            ros::Subscriber m_nav_map;

            std::unique_ptr<NavMapHandler> m_navmap_handler;
            unsigned char** m_grid;

            ros::Subscriber interrupt_sub_;
            boost::mutex mutex;
            
            // Doesn't really need to store the Costmap2D object. Simply has
            // to update the costmap of the heurMgr.
            // std::unique_ptr<costmap_2d::Costmap2DROS> m_costmap_ros;
            // std::unique_ptr<costmap_2d::Costmap2DPublisher> m_costmap_publisher;
    };
}
