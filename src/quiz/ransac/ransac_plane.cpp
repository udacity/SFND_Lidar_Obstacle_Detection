/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @desc Ransac for plane detection
 */

#include "utils.h"
#include "ransac_plane.h"

#include <cstdlib>
#include <math.h>
#include <tuple>
#include <utility>

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}


int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
      CreateData3D(); // addZData(CreateData(), 20);

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  auto seed_inliers = RansacPlane<pcl::PointXYZ>(cloud, 200, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSeed(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  // Add the elected seed (initial line points) !
  cloudSeed->points.push_back(cloud->points[std::get<0>(seed_inliers)]);
  cloudSeed->points.push_back(cloud->points[std::get<1>(seed_inliers)]);
  cloudSeed->points.push_back(cloud->points[std::get<2>(seed_inliers)]);

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (std::get<3>(seed_inliers).count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render point cloud with inliers and outliers
  if (std::get<3>(seed_inliers).size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    renderPointCloud(viewer, cloudSeed, "seed",
                     Color(1, 1, 1)); // seed with bright white

    viewer->addLine(cloud->points[std::get<0>(seed_inliers)],
                    cloud->points[std::get<1>(seed_inliers)], 1, 1, 1, "AB");
    viewer->addLine(cloud->points[std::get<0>(seed_inliers)],
                    cloud->points[std::get<2>(seed_inliers)], 1, 1, 1, "AC");
    viewer->addLine(cloud->points[std::get<1>(seed_inliers)],
                    cloud->points[std::get<2>(seed_inliers)], 1, 1, 1, "BC");
    auto ABxAC =
        cross_product(vector(cloud->points[std::get<0>(seed_inliers)],
                             cloud->points[std::get<1>(seed_inliers)]),
                      vector(cloud->points[std::get<0>(seed_inliers)],
                             cloud->points[std::get<2>(seed_inliers)]));
    viewer->addLine(
        cloud->points[std::get<0>(seed_inliers)],
        translate(cloud->points[std::get<0>(seed_inliers)], unit(ABxAC)), .5,
        .5, .5, "normal");

  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
