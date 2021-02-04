/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = true;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // Create Lidar sensor
  Lidar *lidar = new Lidar(cars, 0);
  auto pointCloud = lidar->scan();
  // renderRays(viewer, lidar->position, pointCloud);
  renderPointCloud(viewer, pointCloud, "lidar");
  // Create point processor
  ProcessPointClouds<pcl::PointXYZ> *ppc =
      new ProcessPointClouds<pcl::PointXYZ>();
  auto segmentCloud = ppc->SegmentPlane(pointCloud, 100, .30);
  renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI> *ppc =
      new ProcessPointClouds<pcl::PointXYZI>();

  // load point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud =
      ppc->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  // filter the point cloud
  auto filteredCloud = ppc->FilterCloud(pointCloud, .2, Eigen::Vector4f(-10, -7, -2, 1), Eigen::Vector4f(20, 7, 2, 1));


  // renderPointCloud(viewer,pointCloud,"pointCloud");

  auto segmentCloud = ppc->SegmentPlane(filteredCloud, 100, .30);

 // we need to filter first (too much points to cluster in the pcd data)

  auto obstacleClusters  = ppc->Clustering(segmentCloud.first, .3, 2, 500);

  // rendering clusters

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1), Color(0, 1, 0), Color(0, 0, 1), Color(0, 1, 1)};
  for (auto clusterCloud : obstacleClusters) {
    renderPointCloud(viewer, clusterCloud,
                     "cluster" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = ppc->BoundingBox(clusterCloud);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
  // rendering roof box (to calibrate the right params.)
  renderBox(viewer, Box{-2., -1.5, -1.0, 2.0, 1.5, .5}, ++clusterId, Color(.5, 0., .5));



  // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud",
                   Color(0.3, 0.3, 0.3));
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  cityBlock(viewer); // simpleHighway(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}