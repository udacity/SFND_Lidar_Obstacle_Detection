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


std::tuple<int, int, int,  std::unordered_set<int>> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// Small sanity checks (to move to tests)
	std::cout << "S. check " << scalar_product(unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(1,0,0))),
				   unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(2,0,0)))) << std::endl;
	std::cout << "S. check norm " << norm(unit(vector(pcl::PointXYZ(0,0,0), pcl::PointXYZ(3, 0, 0)))) << std::endl;

	std::tuple<int, int, int, std::unordered_set<int>> results;


	srand(time(NULL));
	if (cloud->size()>1) {
		// TODO distinguish the case with only 2 points
	for (int iter=0; iter< maxIterations; iter++) {
		std::unordered_set<int> inliers;

		// Randomly sample subset (3 points) to define the plane
		auto pointAIndex = rand() % cloud->size();
		auto pointBIndex = rand() % cloud->size();
		auto pointCIndex = rand() % cloud->size();

		// Measure distance between every point and fitted line
		auto pointA = cloud->at(pointAIndex);
		auto pointB = cloud->at(pointBIndex);
		auto pointC = cloud->at(pointCIndex);

		auto vectorAB = vector(pointA, pointB);
		auto vectorAC = vector(pointA, pointC);
		auto ABxAC = cross_product(vectorAB, vectorAC);

		while (norm(ABxAC) == 0) {
			// should avoid collinearity
			pointBIndex = rand() % cloud->size();
			pointCIndex = rand() % cloud->size();

			pointB = cloud->at(pointBIndex);
			pointC = cloud->at(pointCIndex);

			vectorAB = vector(pointA, pointB);
			vectorAC = vector(pointA, pointC);
			ABxAC = cross_product(vectorAB, vectorAC);
		}

		//  Insert selected seed points (pointA and pointB) indices as inliers
		inliers.insert(pointAIndex);
		inliers.insert(pointBIndex);
		inliers.insert(pointCIndex);

		for (int index = 0; index < cloud->points.size(); index++) {
			// If distance is smaller than threshold count it as inlier
			if (inliers.count(index)>0)
			 continue; // it is one the points A, B defining the line
			// Measure distance between every point and fitted line
			// If distance is smaller than threshold count it as inlier
			if (abs(scalar_product(unit(ABxAC) , vector(pointA, cloud->at(index))) )< distanceTol){
				inliers.insert(index);
			}
		}

		if (inliers.size() > std::get<3>(results).size()) {
			results = std::tie(pointAIndex, pointBIndex, pointCIndex, inliers);
			std::cout << "cross prod. AB x AB (should be 0)" << cross_product(vectorAB,  vectorAB) << std::endl;
			std::cout << "cross prod. AB x AC " << unit(cross_product(vectorAB,  vectorAC)) << std::endl;
			std::cout << "inliers size ->" << inliers.size() << std::endl;
		}
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = addZData(CreateData(), 20);


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	auto seed_inliers = Ransac(cloud, 200, 0.02);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSeed(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	// Add the elected seed (initial line points) !
	cloudSeed->points.push_back(cloud->points[std::get<0>(seed_inliers)]);
	cloudSeed->points.push_back(cloud->points[std::get<1>(seed_inliers)]);
	cloudSeed->points.push_back(cloud->points[std::get<2>(seed_inliers)]);

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(std::get<3>(seed_inliers).count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render point cloud with inliers and outliers
	if(std::get<3>(seed_inliers).size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
		renderPointCloud(viewer,cloudSeed,"seed",Color(1, 1, 1)); // seed with bright white

		viewer->addLine(cloud->points[std::get<0>(seed_inliers)], cloud->points[std::get<1>(seed_inliers)],1,1,1,"AB");
		viewer->addLine(cloud->points[std::get<0>(seed_inliers)], cloud->points[std::get<2>(seed_inliers)],1,1,1,"AC");
		viewer->addLine(cloud->points[std::get<1>(seed_inliers)], cloud->points[std::get<2>(seed_inliers)],1,1,1,"BC");
		auto ABxAC = cross_product(vector(cloud->points[std::get<0>(seed_inliers)], cloud->points[std::get<1>(seed_inliers)]),
								vector(cloud->points[std::get<0>(seed_inliers)], cloud->points[std::get<2>(seed_inliers)]));
		viewer->addLine(cloud->points[std::get<0>(seed_inliers)], translate(cloud->points[std::get<0>(seed_inliers)], unit(ABxAC) ),.5,.5,.5,"normal");




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
