/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cstdlib>
#include <math.h>
#include <utility>
#include <tuple>

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include "utils.h"

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


std::tuple<int, int,  std::unordered_set<int>> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// Small sanity checks (to move to tests)
	std::cout << "S. check " << scalar_product(unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(1,0,0))),
				   unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(2,0,0)))) << std::endl;
	std::cout << "S. check norm" << norm(unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(3, 0, 0)))) << std::endl;

	std::tuple<int, int, std::unordered_set<int>> results;


	srand(time(NULL));
	if (cloud->size()>1) {
		// TODO distinguish the case with only 2 points
	for (int iter=0; iter< maxIterations; iter++) {
		std::unordered_set<int> inliers;

		// Randomly sample subset (2 points) and fit line
		auto pointAIndex = rand() % cloud->size();
		auto pointA = cloud->at(pointAIndex);
		auto pointBIndex = rand() % cloud->size();
		auto pointB = cloud->at(pointBIndex);

		while ((pointAIndex == pointBIndex) || overlapping(pointA, pointB)) {
			// ensure 2 distinct points (by content not by index) re-draw point B
			pointBIndex = rand() % cloud->size();
			pointB = cloud->at(pointBIndex);
			// std::cout << "tie resolved" << std::endl;
		}
		//  Insert selected seed points (pointA and pointB) indices as inliers
		inliers.insert(pointAIndex);
		inliers.insert(pointBIndex);

		// std::cout << "line (points): " << pointA << "," << pointB << std::endl;

		// Measure distance between every point and fitted line
		auto vectorAB = vector(pointA, pointB);

		// std::cout << "Distance B to AB (should be 0)" << distance_to_line(pointB, pointA, vectorAB) << std::endl;
		for (int index = 0; index < cloud->points.size(); index++) {
			// If distance is smaller than threshold count it as inlier
			if (inliers.count(index)>0)
			 continue; // it is one the points A, B defining the line
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (distance_to_line(cloud->at(index), pointA, vectorAB)< distanceTol){
				inliers.insert(index);
			}
		}

		if (inliers.size() > std::get<2>(results).size())
			results = std::tie(pointAIndex, pointBIndex, inliers);
	}

	}
	// Return indicies of inliers from fitted line with most inliers
	return results;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = addZData(CreateData());


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	auto seed_inliers = Ransac(cloud, 200, 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSeed(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	// Add the elected seed (initial line points) !
	cloudSeed->points.push_back(cloud->points[std::get<0>(seed_inliers)]);
	cloudSeed->points.push_back(cloud->points[std::get<1>(seed_inliers)]);

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(std::get<2>(seed_inliers).count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(std::get<2>(seed_inliers).size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
		renderPointCloud(viewer,cloudSeed,"seed",Color(1, 1, 1)); // seed with bright white
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
