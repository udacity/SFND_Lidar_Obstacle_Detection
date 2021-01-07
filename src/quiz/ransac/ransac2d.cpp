/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line/plane fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
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
	srand(time(NULL));

	// For max iterations 
	for (int i=0; i<maxIterations; i++) {
		// Allocate current inlier set
		std::unordered_set<int> inliers;
		// Randomly select two indices. Notice:
		// Unordered set has unique elements therefore it will have no repetitions
		while (inliers.size() < 2) {
			inliers.insert( rand() % cloud->points.size() );
		}
		// Sampled points
		float x1, y1, x2, y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		float A = y1 - y2;
		float B = x2 - x1;
		float C = x1 * y2 - x2 * y1;
		// For all points in cloud
		for (int idx = 0; idx < cloud->points.size(); idx++) {
			// Skip if point was already in result
			if (inliers.count(idx)>0) 
				continue;

			// Measure distance between every point and fitted line
			float x = cloud->points[idx].x;
			float y = cloud->points[idx].y;
			float distance = fabs(A*x + B*y + C) / sqrt(A*A + B*B);
			// If distance is smaller than threshold insert index as inlier
			if (distance <= distanceTol) {
				inliers.insert(idx);
			}
		}
		// If number of inliers is best one, set to result
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	std::cout << "Inlier indices: ";
	for (int inlier : inliersResult)
		 std::cout << inlier << " ";
	std::cout << "\n";
	// Return indices of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	for (int i=0; i<maxIterations; i++) {
		// Allocate current inlier set
		std::unordered_set<int> inliers;
		// Randomly select two indices. Notice:
		// Unordered set has unique elements therefore it will have no repetitions
		while (inliers.size() < 3) {
			inliers.insert( rand() % cloud->points.size() );
		}
		// Sampled points
		pcl::PointXYZ p1, p2, p3;
		auto itr = inliers.begin();
		p1 = cloud->points[*itr];
		itr++;
		p2 = cloud->points[*itr];
		itr++;
		p3 = cloud->points[*itr];
		float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
		float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
		float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
		float D = - (A*p1.x + B*p1.y + C*p1.z);
		// For all points in cloud
		for (int idx = 0; idx < cloud->points.size(); idx++) {
			// Skip if point was already in result
			if (inliers.count(idx)>0) 
				continue;

			// Measure distance between every point and fitted line
			pcl::PointXYZ p = cloud->points[idx];
			float distance = fabs(A*p.x + B*p.y + C*p.z + D) / sqrt(A*A + B*B + C*C);
			// If distance is smaller than threshold insert index as inlier
			if (distance <= distanceTol) {
				inliers.insert(idx);
			}
		}
		// If number of inliers is best one, set to result
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	// std::cout << "Inlier indices: ";
	// for (int inlier : inliersResult)
	// 	 std::cout << inlier << " ";
	// std::cout << "\n";
	// Return indices of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	viewer->setCameraPosition(-5, -5, 5, 1, 1, 0);

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

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


	// Render point cloud with inliers and outliers
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
