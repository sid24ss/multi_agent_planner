#include <multi_agent_planner/Environment.h>
#include <multi_agent_planner_node/EnvInterfaces.h>
#include <ros/ros.h>
#include <memory>

namespace multi_agent_planner_node {
    class Node {
        public:
            Node(ros::NodeHandle nh);
        private:
            std::shared_ptr<multi_agent_planner::Environment> m_env;
            EnvInterfaces m_env_interface;
    };
} 
