/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
int plane_distance_threshold_in_cms = 25;
int min_cluster_size = 10;
int max_cluster_size = 50;
int next_frame = 0;
int maxIterations = 100;
int filter_resolution_cms = 10;
int use_pcl_ransac = 0;
int show_road = false;
int pause_frame = 0;

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
    Lidar* lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_scan = lidar->scan();
    //renderRays(viewer, lidar->position, lidar_scan);
    //renderPointCloud(viewer, lidar_scan, "lidar_scan");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointProcessor.SegmentPlane(lidar_scan, maxIterations, 0.2);
    //renderPointCloud(viewer, segResult.first, "obstacles", Color(1, 0, 0));
    //renderPointCloud(viewer, segResult.second, "plane", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segResult.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
               ProcessPointClouds<pcl::PointXYZI>& pointProcessor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessor.FilterCloud(inputCloud, filter_resolution_cms/100., 
                                             Eigen::Vector4f (-10., -6., -5., 1), 
                                             Eigen::Vector4f ( 50., 6., +5., 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult;
    segResult = pointProcessor.SegmentPlane(filterCloud, maxIterations, plane_distance_threshold_in_cms/100.0);
    //renderPointCloud(viewer, segResult.first, "obstacles", Color(1, 0, 0));
    if (show_road)
        renderPointCloud(viewer, segResult.second, "plane", Color(0, 1, 0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    cloudClusters = pointProcessor.Clustering(segResult.first, 0.20, min_cluster_size, max_cluster_size * 10);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        ++clusterId;

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
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

static void on_trackbar_cluster_size(int , void*)
{
    if (min_cluster_size > max_cluster_size) {
        max_cluster_size = min_cluster_size + 1;
    }
    cv::setTrackbarPos("Min Cluster Size", "Parameters", min_cluster_size);
    cv::setTrackbarPos("Max Cluster Size", "Parameters", max_cluster_size);
}

static void button_callback(int state, void*)
{
    show_road =  !show_road;
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
 
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    int pause = 0;
    namedWindow("Parameters", WINDOW_AUTOSIZE); // Create Window
    cv::createTrackbar( "Segment plane distance threshold in cms", "Parameters", &plane_distance_threshold_in_cms, 200, NULL);
    cv::createTrackbar( "Min Cluster Size", "Parameters", &min_cluster_size, 200, on_trackbar_cluster_size);
    cv::createTrackbar( "Max Cluster Size x 10", "Parameters", &max_cluster_size, 100, on_trackbar_cluster_size);
    cv::createTrackbar( "RanSac Iterations", "Parameters", &maxIterations, 500, NULL);
    cv::createTrackbar( "Voxel Filter Resolution (in cms)", "Parameters", &filter_resolution_cms, 100, NULL);
    cv::createTrackbar( "Use PCL Ransac", "Parameters", &use_pcl_ransac, 1, NULL);
    cv::createTrackbar( "Pause Frame", "Parameters", &pause_frame, 1, NULL);
    cv::createTrackbar( "Show Road", "Parameters", &show_road, 1, button_callback);
    waitKey(1);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        std::cout << (*streamIterator).string() << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd((*streamIterator).string());    
        cityBlock(viewer, pointProcessor, inputCloud);

        if (not pause_frame) {
            streamIterator++;
        }

        // looping
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
        waitKey(1);
    } 
}