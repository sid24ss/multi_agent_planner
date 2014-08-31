#pragma once
#include <multi_agent_planner/ParameterCatalog.h>
#include <sbpl_manipulation_components/occupancy_grid.h>

namespace multi_agent_planner {
    /*! \brief Contains information about the collision map. This tends to be
     * used as a singleton for objects that need discretization information.
     */
    class OccupancyGridUser {
        public:
            static void init(OccupancyGridParams& og_params);
            static std::shared_ptr<sbpl_arm_planner::OccupancyGrid> m_occupancy_grid;
            void getOccupancyGridSize(int& dimX, int& dimY, int&dimZ){m_occupancy_grid->getGridSize(dimX, dimY, dimZ);};
    };
}
