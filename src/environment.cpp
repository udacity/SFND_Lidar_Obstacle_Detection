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


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& pointCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // Filter input PCD to downsample it on a grid and select a box region of interest
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(pointCloud, 0.25, Eigen::Vector4f(-10., -6., -3., 1), Eigen::Vector4f(30., +6., 3., 1));
    // Render filtered PCD on viewer
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // Segment point cloud using RANSAC with 100 iterations and 0.2 m as distance tolerance
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(filterCloud, 100, 0.2);
    // Render obstacle point cloud using red and road using green
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacle point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.4, 10, 500);
    
    // Use rgb colors
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // render each cluster with a different color    
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Also render bounding box
        Box box = pointProcessorI.BoundingBox(cluster);

        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0);

    // Scan environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();

    // Render Lidar rays
    // renderRays(viewer, lidar->position, pointCloud);

    // Render Point Cloud (disabled, below we render road and obstacles separately)
    // renderPointCloud(viewer, pointCloud, "PointCloud");

    // Create point processor
    // 1) On the stack (return actual object)
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    // 2) On the heap (return a pointer Ptr in some way!)
    // ProcessPointClouds<pcl::PointXYZ>* pointProcessorPtr = new ProcessPointClouds<pcl::PointXYZ>();
    
    // Segment point cloud using RANSAC with 100 iterations and 0.2 m as distance tolerance
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(pointCloud, 100, 0.2);
    // Render obstacle point cloud using red and road using green
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacle point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 5,  500);
    
    // Use rgb colors
    std::vector<Color> colors = {Color(1,0,0), Color(1,0,1), Color(0,0,1)};

    // render each cluster with a different color    
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Also render bounding box
        Box box = pointProcessor.BoundingBox(cluster); // normal box
        // BoxQ box = pointProcessor.BoundingBox(cluster);   // PCA box

        renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);

        ++clusterId;
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

    // Create point processor with x, y, z and intensity
    // 1) On the stack (return actual object)
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    // 2) On the heap (return a pointer Ptr in some way!)
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorIPtr = new ProcessPointClouds<pcl::PointXYZI>();
        
    // Load real Point Cloud data
    // pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // Render PCD on viewer
    // renderPointCloud(viewer, pointCloud, "pointCloud");

    // You tell streamPcd a directory that contains all the sequentially ordered pcd files you want to process, 
    // and it returns a chronologically ordered vector of all those file names, called stream. 
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    // Create PCD stream iterator and point cloud
    auto streamItr = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;

    // simpleHighway(viewer);

    // PCL viewer update loop
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        pointCloud = pointProcessorI.loadPcd((*streamItr).string());
        cityBlock(viewer, pointProcessorI, pointCloud);

        // Go to next PCD file
        streamItr++;
        if(streamItr == stream.end())
            streamItr = stream.begin();

        viewer->spinOnce(200);
    } 
}