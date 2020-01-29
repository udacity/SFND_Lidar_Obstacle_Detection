/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
#include <random>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::random_device rd;
	std::mt19937 mt(rd());
	
	// TODO: Fill in this function
	int i = 0, j= 0;
	size_t cloud_size = cloud->size();
	std::uniform_int_distribution<int> dist (0,cloud_size);
	int best_count = 0, current_count = 0;
	int best_i = 0, best_j = 0;
	int x1, y1, x2, y2;
	int a=0, b=0, c=0;

	while(maxIterations > 0){
		maxIterations--;
		i = dist(mt) % cloud_size;
		j = dist(mt) % cloud_size;
		while(i == j){
			j = dist(mt) % cloud_size;
		}

		x1 = cloud->at(i).x;
		x2 = cloud->at(j).x;

		y1 = cloud->at(i).y;
		y2 = cloud->at(j).y;

		a = y1-y2;
		b = x2-x1;
		c = (x1*y2 - x2*y1);

		for(auto &point: *cloud){
			float dist = abs(a*point.x + b * point.y+ c);
			dist = dist / sqrt(a*a + b*b);
			if(dist <= distanceTol){
				current_count++;
			}
		}

		if(current_count > best_count){
			best_count = current_count;
			best_i = i;
			best_j = j;
		}
		current_count = 0;
	}

	x1 = cloud->at(best_i).x;
	x2 = cloud->at(best_j).x;

	y1 = cloud->at(best_i).y;
	y2 = cloud->at(best_j).y;

	a = y1-y2;
	b = x2-x1;
	c = (x1*y2 - x2*y1);
	
	for(int iter = 0; iter<cloud_size; iter++){
		float dist = abs(a* cloud->at(iter).x + b * cloud->at(iter).y + c) / sqrt(a*a + b*b);
		if(dist <= distanceTol){
			inliersResult.insert(iter);
		}
	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier


	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::random_device rd;
	std::mt19937 mt(rd());
	
	// TODO: Fill in this function
	int i = 0, j= 0, k = 0;
	size_t cloud_size = cloud->size();
	std::uniform_int_distribution<int> dist (0,cloud_size);
	int best_count = 0, current_count = 0;
	int best_i = 0, best_j = 0, best_k = 0;
	int x1, y1, z1, x2, y2, z2, x3, y3, z3;
	int a=0, b=0, c=0, d=0;

	while(maxIterations > 0){
		maxIterations--;
		i = dist(mt) % cloud_size;
		j = dist(mt) % cloud_size;
		k = dist(mt) % cloud_size;
		while(i == j){
			j = dist(mt) % cloud_size;
		}
		while(i == k || j == k){
			k = dist(mt) % cloud_size;
		}

		x1 = cloud->at(i).x;
		x2 = cloud->at(j).x;
		x3 = cloud->at(k).x;

		y1 = cloud->at(i).y;
		y2 = cloud->at(j).y;
		y3 = cloud->at(k).y;

		z1 = cloud->at(i).z;
		z2 = cloud->at(j).z;
		z3 = cloud->at(k).z;

		a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		d = -(a*x1 + b*y1 + c*z1);

		for(auto &point: *cloud){
			float dist = abs(a*point.x + b * point.y+ c * point.z + d);
			dist = dist / sqrt(a*a + b*b + c*c);
			if(dist <= distanceTol){
				current_count++;
			}
		}

		if(current_count > best_count){
			best_count = current_count;
			best_i = i;
			best_j = j;
			best_k = k;
		}
		current_count = 0;
	}

	x1 = cloud->at(i).x;
	x2 = cloud->at(j).x;
	x3 = cloud->at(k).x;

	y1 = cloud->at(i).y;
	y2 = cloud->at(j).y;
	y3 = cloud->at(k).y;

	z1 = cloud->at(i).z;
	z2 = cloud->at(j).z;
	z3 = cloud->at(k).z;

	a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
	b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
	c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
	d = -(a*x1 + b*y1 + c*z1);
	
	for(int iter = 0; iter<cloud_size; iter++){
		float dist = abs(a* cloud->at(iter).x + b * cloud->at(iter).y + c *cloud->at(iter).z + d) / sqrt(a*a + b*b + c*c);
		if(dist <= distanceTol){
			inliersResult.insert(iter);
		}
	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier


	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
