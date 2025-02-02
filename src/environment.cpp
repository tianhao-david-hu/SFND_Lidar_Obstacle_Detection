/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    double ground_slope = 0;
    std::shared_ptr<Lidar> p_lidar= std::make_shared<Lidar>(cars,ground_slope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  p_point_cloud = p_lidar->scan();

    // TODO:: Create point processor
    //renderRays(viewer, p_lidar->position, p_point_cloud);
   
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> p_point_cloud_processor = std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = p_point_cloud_processor->SegmentPlane(p_point_cloud,100,0.2);
    //renderPointCloud(viewer, segmentCloud.first,"Obstacle PointCloud",Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second,"Plane PointCloud",Color(0.5,0.5,0.5));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = p_point_cloud_processor->Clustering(segmentCloud.first, 
                                                                                                        1.0 /*dist. tol.*/, 3/*min num points*/,
                                                                                                        30 /*max num points*/);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        p_point_cloud_processor->numPoints(cluster);
        int colorId = clusterId%colors.size();
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[colorId]);

        Box box = p_point_cloud_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId++;

    }
  
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
 
    renderPointCloud(viewer,inputCloud,"inputCloud");

    auto filterCloud = pointProcessorI->FilterCloud(inputCloud, 1.0f/*filterRes*/ , Eigen::Vector4f (-30, -6, -3, 0)/*minPoint*/, Eigen::Vector4f ( 30, 6.5, 4, 1)/*maxPoint*/);
    //renderPointCloud(viewer,filterCloud,"filterCloud");

    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,100,0.2);
    //renderPointCloud(viewer, segmentCloud.first,"Obstacle PointCloud",Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second,"Plane PointCloud",Color(0.5,0.5,0.5));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 
                                                                                                        1.2,//dist. tol.
                                                                                                        5,//min num points
                                                                                                        120);//max num points
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        int colorId = clusterId%colors.size();
        //renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[colorId]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(0,0,1));
        clusterId++;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

    /*cityBlock(viewer,pointProcessorI,inputCloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } */
}