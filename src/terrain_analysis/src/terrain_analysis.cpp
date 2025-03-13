#include <terrain_analysis.h>



TerrainAnalysis::TerrainAnalysis(ros::NodeHandle &nh) : nh_(nh){
    initTerrainAnalysis();
}

TerrainAnalysis::~TerrainAnalysis(){
}

void TerrainAnalysis::initTerrainAnalysis(){
    nh_.param<float>("terrain_analysis/elevationResolution", elevationResolution, 0.02);
    nh_.param<float>("terrain_analysis/elevationExpandX", elevationExpandX, 5.0);
    nh_.param<float>("terrain_analysis/elevationExpandY", elevationExpandY, 5.0);
    nh_.param<float>("terrain_analysis/elevationExpandZMin", elevationExpandZMin, -1.0);
    nh_.param<float>("terrain_analysis/elevationExpandZmax", elevationExpandZmax, 1.5);

    nh_.param<float>("terrain_analysis/cutRobotCenterX", cutRobotCenterX, 0.0);
    nh_.param<float>("terrain_analysis/cutRobotCenterY", cutRobotCenterY, 0.0);
    nh_.param<float>("terrain_analysis/cutRobotCenterZ", cutRobotCenterZ, 0.0);
    nh_.param<float>("terrain_analysis/cutRobotExpandXMin", cutRobotExpandXMin, -0.6);
    nh_.param<float>("terrain_analysis/cutRobotExpandXMax", cutRobotExpandXMax, 0.6);
    nh_.param<float>("terrain_analysis/cutRobotExpandY", cutRobotExpandY, 0.35);
    nh_.param<float>("terrain_analysis/cutRobotExpandZMin", cutRobotExpandZMin, -0.3);
    nh_.param<float>("terrain_analysis/cutRobotExpandZMax", cutRobotExpandZMax, 0.1);
    nh_.param<float>("terrain_analysis/cutRobotExpandZMiddle", cutRobotExpandZMiddle, -0.4);

    nh_.param<float>("terrain_analysis/cutRobotGroundXYResolution", cutRobotGroundXYResolution, 0.05);
    nh_.param<float>("terrain_analysis/cutRobotGroundZResolution", cutRobotGroundZResolution, 0.05);
    nh_.param<float>("terrain_analysis/asGroundProportionThreshold", asGroundProportionThreshold, 0.15);

    nh_.param<float>("terrain_analysis/occupancyGridResolution", occupancyGridResolution, 0.1);
    nh_.param<float>("terrain_analysis/occupancyGridPositionX", occupancyGridPositionX, 0.0);
    nh_.param<float>("terrain_analysis/occupancyGridPositionY", occupancyGridPositionY, 0.0);
    nh_.param<float>("terrain_analysis/occupancyGridPosotionZ", occupancyGridPosotionZ, -1.0);
    nh_.param<float>("terrain_analysis/occupancyGridOrientationX", occupancyGridOrientationX, 0.0);
    nh_.param<float>("terrain_analysis/occupancyGridOrientationY", occupancyGridOrientationY, 0.0);
    nh_.param<float>("terrain_analysis/occupancyGridOrientationZ", occupancyGridOrientationZ, 0.0);
    nh_.param<float>("terrain_analysis/occupancyGridOrientationW", occupancyGridOrientationW, 1.0);

    occupancyGridMsg.header.frame_id = "vehicle"; 
    occupancyGridMsg.info.resolution = occupancyGridResolution;
    occupancyGridMsg.info.origin.position.z = occupancyGridPosotionZ;
    occupancyGridMsg.info.origin.orientation.x = occupancyGridOrientationX;
    occupancyGridMsg.info.origin.orientation.y = occupancyGridOrientationY;
    occupancyGridMsg.info.origin.orientation.z = occupancyGridOrientationZ;
    occupancyGridMsg.info.origin.orientation.w = occupancyGridOrientationW;
    
    subCloud = nh_.subscribe("/camera/depth/color/points", 100, &TerrainAnalysis::CloudCallBack, this);
    subLivox = nh_.subscribe("/livox/lidar", 100, &TerrainAnalysis::CustomMsgCallBack, this);

    pubLidarCloud = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_cloud", 1, true);
    pubNonPassableAreas = nh_.advertise<sensor_msgs::PointCloud2>("/non_passable_areas", 1, true);
    pubGridMap = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1, true);
#ifdef USE_GRIDMAP
    pubElevationMap = nh_.advertise<grid_map_msgs::GridMap>("/elevation_map", 1, true);
    elevationMap = grid_map::GridMap({"elevation"});
    elevationMap.setFrameId("vehicle");
#endif
}

void TerrainAnalysis::CloudCallBack(const sensor_msgs::PointCloud2::Ptr& inputCloud){
    laserTime = inputCloud->header.stamp.toSec(); 

    clock_t start_callback = clock();
    pcl::PointCloud<PointType>::Ptr lidarCloudRaw = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::fromROSMsg(*inputCloud, *lidarCloudRaw);

    pcl::PointCloud<PointType>::Ptr lidarCloud = boost::make_shared<pcl::PointCloud<PointType>>();
    PointType Raw_point;
    for(auto &point : *lidarCloudRaw){
        Raw_point.x = point.z;
        Raw_point.y = -point.x;
        Raw_point.z = -point.y;
        lidarCloud->push_back(Raw_point);
    }
    pcl::PointCloud<PointType>::Ptr lidarCloudCut = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr robotRoundGroundCandiate = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr robotRoundGroundCloud = boost::make_shared<pcl::PointCloud<PointType>>();

    int indexXNum = std::ceil((cutRobotExpandXMax - cutRobotExpandXMin) / cutRobotGroundXYResolution);
    int indexYNum = std::ceil((cutRobotExpandY * 2) / cutRobotGroundXYResolution);
    int indexZNum = std::ceil((cutRobotExpandZMiddle - cutRobotExpandZMin) / cutRobotGroundZResolution);

    std::vector<std::vector<std::vector<std::vector<int>>>>
        robotRoundVoxel(indexXNum, std::vector<std::vector<std::vector<int>>>(indexYNum, std::vector<std::vector<int>>(indexZNum, std::vector<int>())));

    int index = 0;
    for (auto &point : *lidarCloud){
        if (fabs(point.x) > elevationExpandX || fabs(point.y) > elevationExpandY || point.z < elevationExpandZMin || point.z > elevationExpandZmax)
            continue;

        if (point.x > (cutRobotCenterX + cutRobotExpandXMin) && point.x < (cutRobotCenterX + cutRobotExpandXMax) && point.y > (cutRobotCenterY - cutRobotExpandY) && point.y < (cutRobotCenterY + cutRobotExpandY) && point.z > (cutRobotCenterZ + cutRobotExpandZMin) && point.z < (cutRobotCenterZ + cutRobotExpandZMax)){
            if (point.z < cutRobotExpandZMiddle){
                robotRoundGroundCandiate->push_back(point);
                int indexX = (point.x - cutRobotExpandXMin) / cutRobotGroundXYResolution;
                int indexY = (point.y - (-cutRobotExpandY)) / cutRobotGroundXYResolution;
                int indexZ = (point.z - cutRobotExpandZMin) / cutRobotGroundZResolution;
                if (indexX >= 0 && indexX < indexXNum && indexY >= 0 && indexY < indexYNum && indexZ >= 0 && indexZ < indexZNum){
                    robotRoundVoxel[indexX][indexY][indexZ].push_back(index);
                }
                index++;
            }
        }
        else{
            lidarCloudCut->push_back(point);
        }
    }

    for (int i = 0; i < robotRoundVoxel.size(); i++){
        for (int j = 0; j < robotRoundVoxel[i].size(); j++){
            float countPointSize = 0;
            for (int k = 0; k < robotRoundVoxel[i][j].size(); k++){
                countPointSize = countPointSize + robotRoundVoxel[i][j][k].size();
            }
            if (countPointSize > 0){
                int GroundProportionThreshold = -1;
                for (int k = 0; k < robotRoundVoxel[i][j].size(); k++){
                    if ((robotRoundVoxel[i][j][k].size() / countPointSize) > asGroundProportionThreshold){
                        GroundProportionThreshold = k;
                    }
                }

                if (GroundProportionThreshold >= 0){
                    for (int l = 0; l < robotRoundVoxel[i][j][GroundProportionThreshold].size(); l++){
                        (*robotRoundGroundCloud).push_back((*robotRoundGroundCandiate)[robotRoundVoxel[i][j][GroundProportionThreshold][l]]);
                    }
                }
            }
        }
    }

    (*lidarCloudCut) = (*lidarCloudCut) + (*robotRoundGroundCloud);

// -------------------------------------------------------------------------------------------------
#ifdef USE_GRIDMAP
    elevationMap.setGeometry(grid_map::Length(elevationExpandX * 2, elevationExpandY * 2), elevationResolution, grid_map::Position::Zero());
#endif
    int idx_x_map = std::ceil((elevationExpandX * 2) / elevationResolution);
    int idx_y_map = std::ceil((elevationExpandY * 2) / elevationResolution);
    std::vector<std::vector<std::vector<int>>> voxelSpace(idx_x_map, std::vector<std::vector<int>>(idx_y_map, std::vector<int>()));
    for (int i = 0; i < (*lidarCloudCut).size(); i++){
        if (fabs((*lidarCloudCut)[i].x) < elevationExpandX && fabs((*lidarCloudCut)[i].y) < elevationExpandY){
            int indexX = ((*lidarCloudCut)[i].x - (-elevationExpandX)) / elevationResolution;
            int indexY = ((*lidarCloudCut)[i].y - (-elevationExpandY)) / elevationResolution;
            if (indexX >= 0 && indexX < idx_x_map && indexY >= 0 && indexY < idx_y_map){
                voxelSpace[indexX][indexY].push_back(i);
            }
        }
    }
    pcl::PointCloud<PointType>::Ptr lidarCloudElevation(new pcl::PointCloud<PointType>);
    std::vector<std::vector<float>> elevationSpace(idx_x_map, std::vector<float>(idx_y_map, -FLT_MAX));

    for (int i = 0; i < voxelSpace.size(); i++){
        for (int j = 0; j < voxelSpace[i].size(); j++){
            if (voxelSpace[i][j].size() > 0){
                float elevation = 0;
                for (int k = 0; k < voxelSpace[i][j].size(); k++){
                    elevation = elevation + (*lidarCloudCut)[voxelSpace[i][j][k]].z;
                }
                elevation = elevation / voxelSpace[i][j].size();
                elevationSpace[i][j] = elevation;
                PointType pointxyz;
                pointxyz.x = i * elevationResolution - elevationExpandX;
                pointxyz.y = j * elevationResolution - elevationExpandY;
                pointxyz.z = elevation;
                (*lidarCloudElevation).push_back(pointxyz);
#ifdef USE_GRIDMAP
                grid_map::Position position;
                position[0] = i * elevationResolution - elevationExpandX + elevationResolution / 2;
                position[1] = j * elevationResolution - elevationExpandY + elevationResolution / 2;
                grid_map::Index index;
                if (elevationMap.getIndex(position, index)){
                    elevationMap.at("elevation", index) = elevation;
                }
#endif
            }
        }
    }
    float sumX = 0.0, sumY = 0.0;
    pcl::PointCloud<PointTypeRGB>::Ptr nonPassableCloud(new pcl::PointCloud<PointTypeRGB>);
    for (int i = 0; i < elevationSpace.size(); i++){
        for (int j = 0; j < elevationSpace[i].size(); j++){
            if (elevationSpace[i][j] > -FLT_MAX){
                int countNonPassableVoxel = 0;
                for (int k = -2; k <= 2; k++){
                    for (int l = -2; l <= 2; l++){
                        if (k == 0 && l == 0)
                            continue;

                        int indexX = i + k;
                        int indexY = j + l;
                        if (indexX > 0 && indexX < idx_x_map && indexY > 0 && indexY < idx_y_map){
                            if (elevationSpace[indexX][indexY] > -FLT_MAX){
                                if (fabs(elevationSpace[i][j] - elevationSpace[indexX][indexY]) > 0.25){
                                    countNonPassableVoxel++;
                                }
                            }
                        }
                    }
                }
                if (countNonPassableVoxel > 3){
                    PointTypeRGB pointxyzrgb;
                    pointxyzrgb.x = (i - idx_x_map / 2) * elevationResolution;
                    pointxyzrgb.y = (j - idx_y_map / 2) * elevationResolution;
                    pointxyzrgb.z = elevationSpace[i][j];
                    pointxyzrgb.r = 255;
                    pointxyzrgb.g = 0;
                    pointxyzrgb.b = 0;
                    (*nonPassableCloud).push_back(pointxyzrgb);
                    sumX += pointxyzrgb.x;
                    sumY += pointxyzrgb.y;
                }
            }
        }
    }

    occupancyGridMsg.info.width = static_cast<int>(3.5 / occupancyGridResolution);
    occupancyGridMsg.info.height = static_cast<int>(3.5/ occupancyGridResolution);
    occupancyGridMsg.info.origin.position.x = nonPassableCloud->size()!= 0.0? sumX / nonPassableCloud->size() - 0.5 * occupancyGridMsg.info.width * occupancyGridResolution:0.0;
    occupancyGridMsg.info.origin.position.y = nonPassableCloud->size()!= 0.0? sumY / nonPassableCloud->size() - 0.5 * occupancyGridMsg.info.height * occupancyGridResolution:0.0;
    occupancyGridMsg.data.resize(occupancyGridMsg.info.width * occupancyGridMsg.info.height);
    std::fill(occupancyGridMsg.data.begin(), occupancyGridMsg.data.end(), 0);

    for (const auto &point : *nonPassableCloud) {
        int gridX = static_cast<int>((point.x - occupancyGridMsg.info.origin.position.x) / occupancyGridResolution);
        int gridY = static_cast<int>((point.y - occupancyGridMsg.info.origin.position.y) / occupancyGridResolution);

        if (gridX >= 0 && gridX < occupancyGridMsg.info.width && gridY >= 0 && gridY < occupancyGridMsg.info.height) {
            int idx = gridY * occupancyGridMsg.info.width + gridX;
            occupancyGridMsg.data[idx] = 100; 
        }
    }

    pubGridMap.publish(occupancyGridMsg);
#ifdef USE_GRIDMAP
    
    grid_map::GridMapRosConverter::toMessage(elevationMap, elevationMapMsg);
    pubElevationMap.publish(elevationMapMsg);
#endif
    nonPassableCloudMsg.header.stamp = ros::Time().fromSec(laserTime);
    pcl::toROSMsg(*nonPassableCloud, nonPassableCloudMsg);
    nonPassableCloudMsg.header.frame_id = "vehicle";
    pubNonPassableAreas.publish(nonPassableCloudMsg);

    clock_t end_callback = clock();
    double callback_time = ((double)(end_callback - start_callback) / (double)CLOCKS_PER_SEC);
    // ROS_INFO("callback_time: %f ", callback_time);
}

void TerrainAnalysis::CustomMsgCallBack(const terrain_analysis::CustomMsgConstPtr &livoxCloudMsg){
    laserTime = livoxCloudMsg->header.stamp.toSec(); 
    PointType pt_;
    pcl::PointCloud<PointType>::Ptr lidarCloud = boost::make_shared<pcl::PointCloud<PointType>>();
    clock_t start_callback = clock();

    for (unsigned int i = 0; i < livoxCloudMsg->point_num; ++i)
    {
        pt_.x = livoxCloudMsg->points[i].x;
        pt_.y = livoxCloudMsg->points[i].y;
        pt_.z = livoxCloudMsg->points[i].z;
        lidarCloud->push_back(pt_);
    }

    pcl::PointCloud<PointType>::Ptr lidarCloudCut = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr robotRoundGroundCandiate = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::PointCloud<PointType>::Ptr robotRoundGroundCloud = boost::make_shared<pcl::PointCloud<PointType>>();

    int indexXNum = std::ceil((cutRobotExpandXMax - cutRobotExpandXMin) / cutRobotGroundXYResolution);
    int indexYNum = std::ceil((cutRobotExpandY * 2) / cutRobotGroundXYResolution);
    int indexZNum = std::ceil((cutRobotExpandZMiddle - cutRobotExpandZMin) / cutRobotGroundZResolution);

    std::vector<std::vector<std::vector<std::vector<int>>>>
        robotRoundVoxel(indexXNum, std::vector<std::vector<std::vector<int>>>(indexYNum, std::vector<std::vector<int>>(indexZNum, std::vector<int>())));

    int index = 0;
    for (auto &point : *lidarCloud)
    {
        if (fabs(point.x) > elevationExpandX || fabs(point.y) > elevationExpandY || point.z < elevationExpandZMin || point.z > elevationExpandZmax)
            continue;
        if (point.x > (cutRobotCenterX + cutRobotExpandXMin) && point.x < (cutRobotCenterX + cutRobotExpandXMax) && point.y > (cutRobotCenterY - cutRobotExpandY) && point.y < (cutRobotCenterY + cutRobotExpandY) && point.z > (cutRobotCenterZ + cutRobotExpandZMin) && point.z < (cutRobotCenterZ + cutRobotExpandZMax))
        {
            if (point.z < cutRobotExpandZMiddle)
            {
                robotRoundGroundCandiate->push_back(point);
                int indexX = (point.x - cutRobotExpandXMin) / cutRobotGroundXYResolution;
                int indexY = (point.y - (-cutRobotExpandY)) / cutRobotGroundXYResolution;
                int indexZ = (point.z - cutRobotExpandZMin) / cutRobotGroundZResolution;
                if (indexX >= 0 && indexX < indexXNum && indexY >= 0 && indexY < indexYNum && indexZ >= 0 && indexZ < indexZNum)
                {
                    robotRoundVoxel[indexX][indexY][indexZ].push_back(index);
                }
                index++;
            }
        }
        else
        {
            lidarCloudCut->push_back(point);
        }
    }
    for (int i = 0; i < robotRoundVoxel.size(); i++)
    {
        for (int j = 0; j < robotRoundVoxel[i].size(); j++)
        {
            float countPointSize = 0;
            for (int k = 0; k < robotRoundVoxel[i][j].size(); k++)
            {
                countPointSize = countPointSize + robotRoundVoxel[i][j][k].size();
            }
            if (countPointSize > 0)
            {
                int GroundProportionThreshold = -1;
                for (int k = 0; k < robotRoundVoxel[i][j].size(); k++)
                {
                    if ((robotRoundVoxel[i][j][k].size() / countPointSize) > asGroundProportionThreshold)
                    {
                        GroundProportionThreshold = k;
                    }
                }

                if (GroundProportionThreshold >= 0)
                {
                    for (int l = 0; l < robotRoundVoxel[i][j][GroundProportionThreshold].size(); l++)
                    {
                        (*robotRoundGroundCloud).push_back((*robotRoundGroundCandiate)[robotRoundVoxel[i][j][GroundProportionThreshold][l]]);
                    }
                }
            }
        }
    }
    (*lidarCloudCut) = (*lidarCloudCut) + (*robotRoundGroundCloud);

// -------------------------------------------------------------------------------------------------
#ifdef USE_GRIDMAP
    elevationMap.setGeometry(grid_map::Length(elevationExpandX * 2, elevationExpandY * 2), elevationResolution, grid_map::Position::Zero());
#endif
    int idx_x_map = std::ceil((elevationExpandX * 2) / elevationResolution);
    int idx_y_map = std::ceil((elevationExpandY * 2) / elevationResolution);
    std::vector<std::vector<std::vector<int>>> voxelSpace(idx_x_map, std::vector<std::vector<int>>(idx_y_map, std::vector<int>()));
    for (int i = 0; i < (*lidarCloudCut).size(); i++)
    {
        if (fabs((*lidarCloudCut)[i].x) < elevationExpandX && fabs((*lidarCloudCut)[i].y) < elevationExpandY)
        {
            int indexX = ((*lidarCloudCut)[i].x - (-elevationExpandX)) / elevationResolution;
            int indexY = ((*lidarCloudCut)[i].y - (-elevationExpandY)) / elevationResolution;
            if (indexX >= 0 && indexX < idx_x_map && indexY >= 0 && indexY < idx_y_map)
            {
                voxelSpace[indexX][indexY].push_back(i);
            }
        }
    }
    pcl::PointCloud<PointType>::Ptr lidarCloudElevation(new pcl::PointCloud<PointType>);
    std::vector<std::vector<float>> elevationSpace(idx_x_map, std::vector<float>(idx_y_map, -FLT_MAX));
    for (int i = 0; i < voxelSpace.size(); i++)
    {
        for (int j = 0; j < voxelSpace[i].size(); j++)
        {
            if (voxelSpace[i][j].size() > 0)
            {
                float elevation = 0;
                for (int k = 0; k < voxelSpace[i][j].size(); k++)
                {
                    elevation = elevation + (*lidarCloudCut)[voxelSpace[i][j][k]].z;
                }
                elevation = elevation / voxelSpace[i][j].size();
                elevationSpace[i][j] = elevation;
                PointType pointxyz;
                pointxyz.x = i * elevationResolution - elevationExpandX;
                pointxyz.y = j * elevationResolution - elevationExpandY;
                pointxyz.z = elevation;
                (*lidarCloudElevation).push_back(pointxyz);
#ifdef USE_GRIDMAP
                grid_map::Position position;
                position[0] = i * elevationResolution - elevationExpandX + elevationResolution / 2;
                position[1] = j * elevationResolution - elevationExpandY + elevationResolution / 2;
                grid_map::Index index;
                if (elevationMap.getIndex(position, index))
                {
                    elevationMap.at("elevation", index) = elevation;
                }
#endif
            }
        }
    }

    pcl::PointCloud<PointTypeRGB>::Ptr nonPassableCloud(new pcl::PointCloud<PointTypeRGB>);
    float sumX = 0.0, sumY = 0.0;
    for (int i = 0; i < elevationSpace.size(); i++)
    {
        for (int j = 0; j < elevationSpace[i].size(); j++)
        {
            if (elevationSpace[i][j] > -FLT_MAX)
            {
                int countNonPassableVoxel = 0;
                for (int k = -2; k <= 2; k++)
                {
                    for (int l = -2; l <= 2; l++)
                    {
                        if (k == 0 && l == 0)
                            continue;
                        int indexX = i + k;
                        int indexY = j + l;
                        if (indexX > 0 && indexX < idx_x_map && indexY > 0 && indexY < idx_y_map)
                        {
                            if (elevationSpace[indexX][indexY] > -FLT_MAX)
                            {
                                if (fabs(elevationSpace[i][j] - elevationSpace[indexX][indexY]) > 0.25)
                                {
                                    countNonPassableVoxel++;
                                }
                            }
                        }
                    }
                }
                if (countNonPassableVoxel > 2)
                {
                    PointTypeRGB pointxyzrgb;
                    pointxyzrgb.x = (i - idx_x_map / 2) * elevationResolution;
                    pointxyzrgb.y = (j - idx_y_map / 2) * elevationResolution;
                    pointxyzrgb.z = elevationSpace[i][j];
                    pointxyzrgb.r = 255;
                    pointxyzrgb.g = 0;
                    pointxyzrgb.b = 0;
                    (*nonPassableCloud).push_back(pointxyzrgb);
                    sumX += pointxyzrgb.x;
                    sumY += pointxyzrgb.y;
                }
                else
                {
                    PointType ground_point;
                    ground_point.x = (i - idx_x_map / 2) * elevationResolution;
                    ground_point.y = (j - idx_y_map / 2) * elevationResolution;
                    ground_point.z = elevationSpace[i][j];
                    robotRoundGroundCloud->push_back(ground_point);
                }
            }
        }
    }

    occupancyGridMsg.info.width = static_cast<int>(5.5 / occupancyGridResolution);
    occupancyGridMsg.info.height = static_cast<int>(5.5/ occupancyGridResolution);
    occupancyGridMsg.info.origin.position.x = nonPassableCloud->size()!= 0? sumX / nonPassableCloud->size() - 0.5 * occupancyGridMsg.info.width * occupancyGridResolution:0.0;
    occupancyGridMsg.info.origin.position.y = nonPassableCloud->size()!= 0? sumY / nonPassableCloud->size() - 0.5 * occupancyGridMsg.info.height * occupancyGridResolution:0.0;
    occupancyGridMsg.data.resize(occupancyGridMsg.info.width * occupancyGridMsg.info.height);
    std::fill(occupancyGridMsg.data.begin(), occupancyGridMsg.data.end(), 0);

    for (const auto &point : *nonPassableCloud) {
        int gridX = static_cast<int>((point.x - occupancyGridMsg.info.origin.position.x) / occupancyGridResolution);
        int gridY = static_cast<int>((point.y - occupancyGridMsg.info.origin.position.y) / occupancyGridResolution);

        if (gridX >= 0 && gridX < occupancyGridMsg.info.width && gridY >= 0 && gridY < occupancyGridMsg.info.height) {
            int idx = gridY * occupancyGridMsg.info.width + gridX;
            occupancyGridMsg.data[idx] = 100; 
        }
    }

    pubGridMap.publish(occupancyGridMsg);

#ifdef USE_GRIDMAP
    grid_map::GridMapRosConverter::toMessage(elevationMap, elevationMapMsg);
    pubElevationMap.publish(elevationMapMsg);
#endif
    lidarCloudCutMsg.header.stamp = ros::Time().fromSec(laserTime);
    pcl::toROSMsg(*robotRoundGroundCloud, lidarCloudCutMsg);
    lidarCloudCutMsg.header.frame_id = "vehicle";
    
    pubLidarCloud.publish(lidarCloudCutMsg);
    nonPassableCloudMsg.header.stamp = ros::Time().fromSec(laserTime);
    pcl::toROSMsg(*nonPassableCloud, nonPassableCloudMsg);
    nonPassableCloudMsg.header.frame_id = "vehicle";
    pubNonPassableAreas.publish(nonPassableCloudMsg);

    clock_t end_callback = clock();
    double callback_time = ((double)(end_callback - start_callback) / (double)CLOCKS_PER_SEC);
    // ROS_INFO("callback_time: %f ", callback_time);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "terrain_analysis");
    ros::NodeHandle nh;
    TerrainAnalysis local_map(nh);

    ros::spin();
    return 0;
}
