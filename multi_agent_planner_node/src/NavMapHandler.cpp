#include <multi_agent_planner_node/NavMapHandler.h>

using namespace multi_agent_planner_node;
// using namespace multi_agent_planner;

NavMapHandler::NavMapHandler(ros::NodeHandle nh)
    : m_nodehandle(nh)
{
    // m_costmap_publisher.reset(new costmap_2d::Costmap2DPublisher(m_nodehandle,1,"/map"));
}

void NavMapHandler::addCostmap(std::string name)
{
    m_costmaps_ros.emplace_back(
        // std::unique_ptr<costmap_2d::Costmap2DROS>(
                new costmap_2d::Costmap2DROS(name, m_tf)
            // )
        );
    m_mapping.insert(
        std::unordered_map<std::string, int>::value_type(
                name, m_costmaps_ros.size()-1
            )
        );
}

void NavMapHandler::getInflatedMap(std::string name,
                                std::vector<unsigned char>& data)
{
    // Get the underlying costmap in the cost_map object.
    // Publish for visualization. Publishing is done for the entire (uncropped) costmap.
    costmap_2d::Costmap2D cost_map;
    int costmap_idx = m_mapping.at(name);
    m_costmaps_ros[costmap_idx]->getCostmapCopy(cost_map);

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
    cost_map.reinflateWindow(dimX*m_map_info.resolution/2, dimY*m_map_info.resolution/2, dimX*m_map_info.resolution, dimY*m_map_info.resolution);

    std::vector<unsigned char> uncropped_map;
    for (unsigned int j = 0; j < cost_map.getSizeInCellsY(); ++j)
    {
        for (unsigned int i = 0; i < cost_map.getSizeInCellsX(); ++i)
        {
            uncropped_map.push_back(cost_map.getCost(i,j));
        }
    }

    m_costmap_publishers.emplace_back(
            new costmap_2d::Costmap2DPublisher(
                ros::NodeHandle(m_nodehandle, name),1,"/map"
            )
        );
    m_costmap_publishers.back()->updateCostmapData(cost_map, m_costmaps_ros[costmap_idx]->getRobotFootprint());

    // Publish the full costmap
    // topic : /multi_agent_planner_node/inflated_obstacles (RViz: Grid
    // Cells)
    m_costmap_publishers.back()->publishCostmap();
    // topic : /multi_agent_planner_node/robot_footprint (RViz: polygon)
    // m_costmap_publisher->publishFootprint();

    // TODO: Check if this is the right thing to do : Take the resolution from
    // the map for the occupancy grid's values.
    double width = dimX*m_map_info.resolution;
    double height = dimY*m_map_info.resolution;
    
    crop2DMap(uncropped_map, 0, 0, width, height, data);
}

void NavMapHandler::loadNavMap(const nav_msgs::OccupancyGridPtr& map)
{
    m_map_info = map->info;
}

void NavMapHandler::crop2DMap(const std::vector<unsigned char>& v,
                              double new_origin_x, double new_origin_y,
                              double width, double height,
                              std::vector<unsigned char>& cropped_map)
{
    ROS_DEBUG_NAMED(CONFIG_LOG, "to be cropped to : %f (width), %f (height)", width, height);
    ROS_DEBUG_NAMED(CONFIG_LOG, "m_map_info : %d width, %d height", m_map_info.width, m_map_info.height);
    // make a grid out of the data
    std::vector<std::vector<unsigned char> > tmp_map(m_map_info.height);
    for (unsigned int i=0; i < m_map_info.height; i++){
        for (unsigned int j=0; j < m_map_info.width; j++){
            tmp_map[i].push_back(v[i*m_map_info.width+j]);
        }
    }

    // compute the new origin and the new width, height
    double res = m_map_info.resolution;
    ROS_DEBUG_NAMED(CONFIG_LOG, "resolution : %f", res);
    int new_origin_x_idx = (new_origin_x-m_map_info.origin.position.x)/res;
    int new_origin_y_idx = (new_origin_y-m_map_info.origin.position.y)/res;
    int new_width = static_cast<int>((width/res) + 1 + 0.5);
    int new_height = static_cast<int>((height/res) + 1 + 0.5);
    ROS_DEBUG_NAMED(CONFIG_LOG, "new origin: %d %d, new_width and new_height: %d %d",
                              new_origin_x_idx, new_origin_y_idx, new_width, 
                              new_height);
    ROS_DEBUG_NAMED(CONFIG_LOG, "size of map %lu %lu", tmp_map.size(), 
                                                     tmp_map[0].size());

    // make the new map
    std::vector<std::vector<unsigned char> > new_map(new_height);
    int row_count = 0;
    for (int i=new_origin_y_idx; i < new_origin_y_idx + new_height; i++){
        for (int j=new_origin_x_idx; j < new_origin_x_idx + new_width; j++){
            new_map[row_count].push_back(tmp_map[i][j]);
        }
        row_count++;
    }

    // output to an array
    cropped_map.clear();
    for (size_t i=0; i < new_map.size(); i++){
        for (size_t j=0; j < new_map[i].size(); j++) {
            cropped_map.push_back(new_map[i][j]);
        }
    }
    ROS_DEBUG_NAMED(CONFIG_LOG, "size of final map: %lu", cropped_map.size());
}