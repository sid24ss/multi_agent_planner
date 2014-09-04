#pragma once
#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/OccupancyGridUser.h>
#include <multi_agent_planner/ParameterCatalog.h>
#include <arm_navigation_msgs/CollisionMap.h>
#include <Eigen/Core>
#include <memory>

namespace multi_agent_planner {
    /**
     * @brief collision checker for the planner
     * @details does collision checking on GraphState, SwarmState, RobotState,
     * and the motion primitives
     */
    class CollisionSpaceMgr : public OccupancyGridUser {
        public:
            CollisionSpaceMgr(const RobotDescriptionParams& params);
            bool isValid(const RobotState& robot_state) const;
            bool isValid(const SwarmState& swarm_state) const;
            bool isValidSuccessor(const GraphState& successor) const;

            void updateMap(const arm_navigation_msgs::CollisionMap& map);
            bool loadMap(const std::vector<Eigen::Vector3d>& points);

        private:
            double m_robot_radius;
            bool checkCollision(const RobotState& robot_state) const;
    };
    typedef std::shared_ptr<CollisionSpaceMgr> CSpaceMgrPtr;
}
