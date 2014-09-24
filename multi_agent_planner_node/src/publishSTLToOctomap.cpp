#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Utilities.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <leatherman/utils.h>
#include <leatherman/file.h>
#include <leatherman/binvox.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <ctime>

using namespace boost::filesystem;
using namespace std;
using namespace multi_agent_planner;

const double dimX = 10;
const double dimY = 10;

bool getVoxelsFromMesh(std::string resource, 
        geometry_msgs::Pose pose, 
        std::vector<std::vector<double> > &voxels){
    std::vector<int32_t> triangles;
    std::vector<geometry_msgs::Point> vertices, scaled_vertices;

    // get STL version of file
    std::string mesh_type = leatherman::getExtension(resource);
    std::string stl_resource = resource;
    if(!mesh_type.empty()){
        if(mesh_type.compare("dae") == 0)
            stl_resource = leatherman::replaceExtension(resource, "stl");
        ROS_DEBUG("Collada file found. Will try to use the STL version instead. (%s)", 
                 resource.c_str());
        ROS_DEBUG("STL filename: %s", stl_resource.c_str());
    }

    // get triangles
    geometry_msgs::Vector3 temp_scale;
    temp_scale.x = 1.0;
    temp_scale.y = 1.0;
    temp_scale.z = 1.0;
    if(!leatherman::getMeshComponentsFromResource(stl_resource, temp_scale, 
                                                  triangles, vertices))
        return false;

    // scale the vertices
    double scale = leatherman::getColladaFileScale(resource);
    leatherman::scaleVertices(vertices, scale, scale, scale, scaled_vertices);
    ROS_DEBUG("Scaled the vertices uniformly by a factor of %0.3f.", scale);

    // voxelize!
    sbpl::Voxelizer::voxelizeMesh(scaled_vertices, triangles, pose, 0.02, voxels, false);

    return true;
}

void addCuboid(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, double X, double Y, double Z, double dimX, double dimY, double dimZ, bool fill = true){
    // Add a cuboid of size dimX, dimY, dimZ, with the bottom left being the
    // reference point: X, Y, Z
    // double dimX, dimY, dimZ;
    // double X, Y, Z;

    std::vector<std::vector<double> > voxels;
    sbpl::Voxelizer::voxelizeBox(dimX, dimY, dimZ, 0.02, voxels, fill);
    size_t currentPCLCloudSize = pclCloud->points.size();
    pclCloud->points.resize(currentPCLCloudSize + voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i)
    {
        pclCloud->points[currentPCLCloudSize + i].x = voxels[i][0] + (X +
            dimX/2);
        pclCloud->points[currentPCLCloudSize + i].y = voxels[i][1] + (Y +
            dimY/2);
        pclCloud->points[currentPCLCloudSize + i].z = voxels[i][2] + (Z +
            dimZ/2);
    }
}

void addRandomObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, int
    num_obstacles, unsigned int seed){
    ros::NodeHandle nh;

    // Add the surface - these are generated only within these bounds.
    double obstacleSizeXMax = 2;
    double obstacleSizeXMin = 0.1;

    double obstacleSizeYMax = 2;
    double obstacleSizeYMin = 0.1;

    // 0.8 is here for extra padding, because of the bounds of the environment.
    double obstacleBoundsXMin = 0.8 + obstacleSizeXMax/2;
    double obstacleBoundsXMax = dimX - 0.8 - obstacleSizeXMax/2;
    
    double obstacleBoundsYMin = 0.8 + obstacleSizeYMax/2;
    double obstacleBoundsYMax = dimY - 0.8 - obstacleSizeYMax/2;

    srand(seed);

    for (int i = 0; i < num_obstacles; ++i){
        // Generate size
        double obs_dimX = randomDouble(obstacleSizeXMin, obstacleSizeXMax);
        double obs_dimY = randomDouble(obstacleSizeYMin, obstacleSizeYMax);
        double obs_dimZ = 0.2;

        // generate position and adjust to center
        double X = randomDouble(obstacleBoundsXMin, obstacleBoundsXMax) - obs_dimX/2;
        double Y = randomDouble(obstacleBoundsYMin, obstacleBoundsYMax) - obs_dimY/2;
        double Z = 0.0;

        // Now, we can add this surface.
        addCuboid(pclCloud, X, Y, Z, obs_dimX, obs_dimY, obs_dimZ, false);
    }
}

void addStartStateRegionToParamServer(){
    // Sets the start state regions to the param server for experiments.
    // Note: Setting only one start region for now.
    ros::NodeHandle nh;
    double X = 0.5;
    double Y = 0.5;
    double Z = 0.1;
    double dimX = 8;
    double dimY = 5;
    double dimZ = 0.0;
    nh.setParam("/multi_agent_planner_node/experiments/start_region_x", X);
    nh.setParam("/multi_agent_planner_node/experiments/start_region_y", Y);
    nh.setParam("/multi_agent_planner_node/experiments/start_region_z", Z);
    nh.setParam("/multi_agent_planner_node/experiments/start_region_dimx",
        dimX);
    nh.setParam("/multi_agent_planner_node/experiments/start_region_dimy",
        dimY);
    nh.setParam("/multi_agent_planner_node/experiments/start_region_dimz",
        dimZ);
}

void addEnvironmentComponents(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud){
    // first corridor
    addCuboid(pclCloud, 0, 0, 0, 0.04, 4, 0.5, true);
    addCuboid(pclCloud, 2, 0, 0, 0.04, 2.0, 0.5, true);
    // second corridor
    addCuboid(pclCloud, 0, 4, 0, 5.00, 0.04, 0.5, true);
    addCuboid(pclCloud, 2, 2, 0, 5.00, 0.04, 0.5, true);
    // third corridor
    addCuboid(pclCloud, 5, 4, 0, 0.04, 1.00, 0.5, true);
    addCuboid(pclCloud, 7, 2, 0, 0.04, 5.00, 0.5, true);
    // fourth corridor
    addCuboid(pclCloud, 0, 5, 0, 5.00, 0.04, 0.5, true);
    addCuboid(pclCloud, 0, 7, 0, 7.00, 0.04, 0.5, true);
}

vector<Eigen::Vector3d> getVoxelsFromFile(std::string filename){
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_environment", 1);
    path p(filename);
    if (!exists(p)){
        ROS_ERROR("Couldn't find %s to load in the collision map!", 
                  filename.c_str());
    }

    ROS_DEBUG("Reading stl file %s",filename.c_str());

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    std::vector<std::vector<double> > voxels;
    getVoxelsFromMesh(filename, pose, voxels);

    std::vector<Eigen::Vector3d> points;
    double maxx = -1;
    double maxy = -1;
    double maxz = -1;
    double minx = 100000;
    double miny = 100000;
    double minz = 100000;
    for(unsigned int i=0; i<voxels.size(); i++){
        if(voxels[i][0]>maxx)
            maxx = voxels[i][0];
        if(voxels[i][1]>maxy)
            maxy = voxels[i][1];
        if(voxels[i][2]>maxz)
            maxz = voxels[i][2];
        if(voxels[i][0]<minx)
            minx = voxels[i][0];
        if(voxels[i][1]<miny)
            miny = voxels[i][1];
        if(voxels[i][2]<minz)
            minz = voxels[i][2];
    }
    for(unsigned int i=0; i<voxels.size(); i++){
        //TODO: this offset should be read in from somewhere!
        Eigen::Vector3d p(voxels[i][0]-minx,
                voxels[i][1]-miny,
                voxels[i][2]-minz-0.32);
        points.push_back(p);
    }

    //mapsize_x = maxx-minx;
    //mapsize_y = maxy-miny;

    ROS_DEBUG("bounds (%f, %f, %f) to (%f, %f, %f)",
            minx,miny,minz,maxx,maxy,maxz);

    //make a point cloud for visualization
    ROS_INFO("visualizing point cloud of size %lu", points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclCloud->points.resize(points.size());
    for(unsigned int i=0; i<points.size(); i++){
        pclCloud->points[i].x = points[i][0];
        pclCloud->points[i].y = points[i][1];
        pclCloud->points[i].z = points[i][2];
    }

    bool randomize_environment;
    ph.param("randomize_environment",randomize_environment,true);
    ROS_WARN("randomize environment set to %d", static_cast<int>(randomize_environment));
    bool add_random_obstacles;
    ph.param("add_random_obstacles", add_random_obstacles, true);
    if(add_random_obstacles){
        if(!randomize_environment){
            string environment_obstacles_file;
            ph.param<string>("experiments/environment_obstacles_file",environment_obstacles_file,"");
            if(environment_obstacles_file.empty()){
              throw new std::runtime_error("rosparam randomize_environment was set to false, but then environment_obstacles_file was not set!");
            }
            FILE* fin = fopen(environment_obstacles_file.c_str(), "r");
            if(!fin){
              ROS_ERROR("environment_obstacles_file did not lead to a file");
              exit(1);
            }
            unsigned int seed;
            int num_obstacles;
            bool success = true;
            success &= fscanf(fin,"randomSeed: %u\n",&seed) == 1;
            success &= fscanf(fin,"num_obstacles: %d\n",&num_obstacles) == 1;
            if(!success){
              throw new std::runtime_error("envt obstacle param file formatted incorrectly");
            }
            addRandomObstacles(pclCloud, num_obstacles, seed);
            ROS_WARN("loaded environment from file (seed=%d) with %d obstacles",seed,num_obstacles);
        }
        else {
            string out_path;
            ph.param<string>("out_path",out_path,"");
            if(out_path.empty()){
              throw new std::runtime_error("No path specified to write to!");
            }
            nh.setParam("/multi_agent_planner_node/experiments/out_path", out_path);
            out_path.append("/env.yaml");
            int num_obstacles_min = 8;
            int num_obstacles_max = 12;
            int num_obstacles = num_obstacles_min + 
                                rand()%(num_obstacles_max - num_obstacles_min + 1);
            unsigned int seed = static_cast<unsigned int>(time(NULL));
            ROS_WARN("generating random environment (seed=%d) with %d obstacles",seed,num_obstacles);
            addRandomObstacles(pclCloud, num_obstacles, seed);
            ROS_WARN("writing env.yaml file!");
            FILE* fout = fopen(out_path.c_str(), "w");
            fprintf(fout,"randomSeed: %u\n",seed);
            fprintf(fout,"num_obstacles: %d\n",num_obstacles);
            fclose(fout);
        }
    } else {
        addEnvironmentComponents(pclCloud);
    }
    // addStartStateRegionToParamServer();

    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg (*pclCloud, pc);
    pc.header.frame_id = "/map";
    pc.header.stamp = ros::Time::now();
    sleep(1);
    pcl_pub.publish(pc);

    return points;
}


bool isValidPath(std::string filename){
    path input_path(filename.c_str());
    if (exists(input_path)){
        ROS_DEBUG("Pulling in data from %s", filename.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to find file '%s'", filename.c_str());
        return false;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stl_to_octomap");
    if (argc != 2){
        ROS_ERROR("missing filename for map!");
        exit(1);
    }
    //if (isValidPath(string(argv[1]))){
        getVoxelsFromFile(string(argv[1]));
    //}
    return 0;
}
