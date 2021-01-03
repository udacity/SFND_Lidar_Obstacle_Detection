/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
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
	while (numOutliers--)
	{
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

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

float CalculatePointDistanceToLine(float a, float b, float c, float x, float y)
{
	float distance = fabs(a * x + b * y + c) / sqrt(a * a + b * b);
	return distance;
}

float CalculatePointDistanceToPlane(float a, float b, float c, float d, float x, float y, float z)
{
	// * d=∣A∗x+B∗y+C∗z+D∣/sqrt(A2+B2+C2).
	return fabs(a * x + b * y + c * z + d) / sqrt(a * a + b * b + c * c);

}
pcl::PointXYZ CrossProduct(pcl::PointXYZ p1, pcl::PointXYZ p2){
	pcl::PointXYZ resultVector;
	resultVector.x = p1.y * p2.z - p1.z * p2.y;
	resultVector.y = p1.z * p2.x - p1.x * p2.z;
	resultVector.z = p1.x * p2.y - p1.y * p2.x;
	return resultVector;
}

void GenerateRandomPointsIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, unsigned int *p1, unsigned int *p2, unsigned int *p3)
{
	*p1 = rand() % cloud->points.size();
	*p2 = rand() % cloud->points.size();
	*p3 = rand() % cloud->points.size();
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;

	srand(time(NULL));

	// TODO: Fill in this function
	//std::pair<unsigned int, unsigned int> randomPointsIndices;
	unsigned int *p1, *p2, *p3;
	unsigned int p1Index, p2Index, p3Index;
	p1 = &p1Index;
	p2 = &p2Index;
	p3 = &p3Index;

	// outer loop
	for (unsigned i = 0; i < maxIterations; i++)
	{

		float distance;
		std::unordered_set<int> temp;
		// Randomly sample subset and fit line
		GenerateRandomPointsIndices(cloud, p1, p2, p3);
		pcl::PointXYZ point1, point2, point3;
		point1 = cloud->points[p1Index];
		point2 = cloud->points[p2Index];
		point3 = cloud->points[p3Index];
		//inner loop
		for (unsigned i = 0; i < cloud->points.size(); i++)
		{

			float a, b, c, d;

			/*
			 * line = aX + bY + cZ + D
			 * v1 = <x2-x1, y2-y1, z2-z1>
			 * v2 = <x3-x1, y3-y1. z3-z1>
			 * v1 x v2 = <i,j,k>
			 * 
			 * a = i
			 * b = j
			 * c = k
			 * d = -(ix1 + jy1 + kz1)
			 */
			
			pcl::PointXYZ v1(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z );
			pcl::PointXYZ v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z );
			
			pcl::PointXYZ normalV= CrossProduct(v1, v2);
			a = normalV.x; b = normalV.y; c = normalV.z; 
			d = -(normalV.x * v1.x + normalV.y * v1.y + normalV.z * v1.z );

			distance = CalculatePointDistanceToPlane(a, b,  c,  d,  cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
			// a = point1.y - point2.y;
			// b = point1.x - point2.x;
			// c = point1.x * point2.y - point1.y * point2.x;

			// // Measure distance between every point and fitted line
			// // If distance is smaller than threshold count it as inlier
			// distance = CalculatePointDistanceToLine(a, b, c, cloud->points[i].x, cloud->points[i].y);

			if (distance <= distanceTol)
			{
				temp.insert(i);
			}
		}

		if (temp.size() > inliersResult.size())
		{
			inliersResult = temp;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 200, 0.7);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
