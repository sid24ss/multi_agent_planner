#pragma once
#include <multi_agent_planner/Heuristics/HeuristicMgr.h>
#include <multi_agent_planner/CollisionSpaceMgr.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/message_filter.h>

namespace multi_agent_planner_node {
    typedef tf::MessageFilter<arm_navigation_msgs::CollisionMap> CollisionMapMsgFilter;
    typedef std::vector<Eigen::Vector3d> VoxelList;
    class CollisionSpaceInterface {
        public:
            CollisionSpaceInterface(multi_agent_planner::CSpaceMgrPtr, 
                                    multi_agent_planner::HeuristicMgrPtr);
            bool bindCollisionSpaceToTopic(std::string topic_name, 
                                           tf::TransformListener& tf, 
                                           std::string ref_frame);
            void update3DHeuristicMaps();
            void update2DHeuristicMaps(std::vector<unsigned char>& data);
            void getOccupancyGridSize(int& dimX, int& dimY, int&dimZ){
                m_cspace_mgr->getOccupancyGridSize(dimX, dimY, dimZ); };
            inline void setHeuristicMgr(multi_agent_planner::HeuristicMgrPtr
                heur_mgr){
                m_heur_mgr = heur_mgr;
            }
            inline multi_agent_planner::CSpaceMgrPtr getCollisionSpace(){
                return m_cspace_mgr;};

            boost::mutex* mutex;
        private:
            std::string m_ref_frame;
            void mapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map);
            message_filters::Subscriber<arm_navigation_msgs::CollisionMap> m_collision_map_subscriber;
            std::shared_ptr<CollisionMapMsgFilter> m_collision_map_filter;
            multi_agent_planner::CSpaceMgrPtr m_cspace_mgr;
            ros::NodeHandle m_nodehandle;
            ros::Publisher m_cmap_pub;
            multi_agent_planner::HeuristicMgrPtr m_heur_mgr;
    };
}
