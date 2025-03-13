#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "terrain_analysis/CustomMsg.h"


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/octree/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transformation_from_correspondences.h>

#include <nav_msgs/OccupancyGrid.h>

#ifdef USE_GRIDMAP
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#endif

typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGB PointTypeRGB;

class TerrainAnalysis
{
public:
    TerrainAnalysis(ros::NodeHandle &nh);
    void initTerrainAnalysis();
    void CloudCallBack(const sensor_msgs::PointCloud2::Ptr& input_cloud);
    void CustomMsgCallBack(const terrain_analysis::CustomMsgConstPtr& livox_cloud_msg);
    ~TerrainAnalysis();
private:
    ros::NodeHandle nh_;

    ros::Subscriber subCloud;
    ros::Subscriber subLivox;

    ros::Publisher pubLidarCloud;
    ros::Publisher pubElevationMap;
    ros::Publisher pubNonPassableAreas;
    ros::Publisher pubGridMap;

    nav_msgs::OccupancyGrid occupancyGridMsg;
    sensor_msgs::PointCloud2 lidarCloudCutMsg;
    sensor_msgs::PointCloud2 nonPassableCloudMsg;
    
    grid_map_msgs::GridMap elevationMapMsg;
    grid_map::GridMap elevationMap;

    float  elevationResolution = 0.02;
    float  elevationExpandX = 5.0;
    float  elevationExpandY = 5.0;
    float  elevationExpandZMin = -1.0;
    float  elevationExpandZmax = 1.5;

    float  cutRobotCenterX = 0.0;
    float  cutRobotCenterY = 0.0;
    float  cutRobotCenterZ = 0.0;
    float  cutRobotExpandXMin = -0.6;
    float  cutRobotExpandXMax = 0.6;
    float  cutRobotExpandY = 0.35;
    float  cutRobotExpandZMin = -0.3;
    float  cutRobotExpandZMax = 0.1;
    float  cutRobotExpandZMiddle = 0.1;

    float  cutRobotGroundXYResolution = 0.05;
    float  cutRobotGroundZResolution = 0.05;
    float  asGroundProportionThreshold = 0.15;

    float  occupancyGridResolution = 0.1;
    float  occupancyGridPositionX = 0.0;
    float  occupancyGridPositionY = -1.75;
    float  occupancyGridPosotionZ = -1.0;
    float  occupancyGridOrientationX = 0.0;
    float  occupancyGridOrientationY = 0.0;
    float  occupancyGridOrientationZ = 0.0;
    float  occupancyGridOrientationW = 1.0;
    
    double laserTime = 0.0;
};