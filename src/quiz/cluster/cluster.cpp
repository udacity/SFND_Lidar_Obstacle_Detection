/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  for(int i = 0; i < points.size(); i++)
  {
    pcl::PointXYZ point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = 0;

    cloud->points.push_back(point);

  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

  if(node!=NULL)
  {
    Box upperWindow = window;
    Box lowerWindow = window;
    // split on x axis
    if(depth%2==0)
    {
      viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    }
    // split on y axis
    else
    {
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }
    iteration++;

    render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
    render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


  }

}

void clusterHelper(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& cluster,std::vector<bool>& processed, KdTree3d* tree,float distanceTol)
{

  processed[i] = true;
  cluster.push_back(i);


  std::vector<float> p = {
    cloud->points[i].x,
    cloud->points[i].y,
    cloud->points[i].z,
  };
  std::vector<int> nearest = tree->search(p,distanceTol);

  for(int id: nearest)
  {

    if(!processed[id])
      clusterHelper(id,cloud,cluster, processed, tree, distanceTol);
  }
}

std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, KdTree3d* tree, float distanceTol)
{

  std::vector<std::vector<int>> clusters;

  std::vector<bool> processed(cloud->points.size(),false);

  
  int i =0;
  while(i < cloud->points.size())
  {
    if(processed[i])
    {
      i++;
      continue;
    }

    std::vector<int> cluster;
    clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }

  return clusters;

}

int main ()
{

  // Create viewer
  /*
  Box window;
  window.x_min = -10;
  window.x_max =  10;
  window.y_min = -10;
  window.y_max =  10;
  window.z_min =   0;
  window.z_max =   0;
  */
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  /*
  std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
  //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  renderPointCloud(viewer,cloud,"data");

  KdTree3d* tree3d = new KdTree3d;

  for (int i=0; i<cloud->points.size(); i++) 
  {
    std::vector<float> p = {
      cloud->points[i].x,
      cloud->points[i].y,
      cloud->points[i].z
    };
    tree3d->insert(p,i); 
  }

  /*
  int it = 0;
  render2DTree(tree->root,viewer,window, it);

  std::cout << "Test Search" << std::endl;
  std::vector<int> nearby = tree3d->search({-6,7,2},3.0);
  for(int index : nearby)
    std::cout << index << ",";
  std::cout << std::endl;

  */
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  //
  std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree3d, 1.0);
  //
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for(std::vector<int> cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(int indice: cluster)
      clusterCloud->points.push_back(pcl::PointXYZ(
            cloud->points[indice].x,
            cloud->points[indice].y,
            cloud->points[indice].z));
    renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
    ++clusterId;
  }
  if(clusters.size()==0)
    renderPointCloud(viewer,cloud,"data");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }

}
