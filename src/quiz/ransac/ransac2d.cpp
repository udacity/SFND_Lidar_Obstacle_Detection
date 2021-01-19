/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

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
	// TODO: Fill in this function
    int idx1, idx2, idx3;
    int maxInliers {0};
    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    float coeffA, coeffB, coeffC, coeffD, denom, distance;
    std::unordered_set<int> buffer;
    std::size_t points_size = cloud->size();
    for(int i {0}; i < maxIterations; i++ ) {
        idx1 = rand() % points_size;
        x1 = cloud->points.at(idx1).x;
        y1 = cloud->points.at(idx1).y;
        z1 = cloud->points.at(idx1).z;
        idx2 = rand() % points_size;
        x2 = cloud->points.at(idx2).x;
        y2 = cloud->points.at(idx2).y;
        z2 = cloud->points.at(idx3).z;
        idx3 = rand() % points_size;
        x3 = cloud->points.at(idx3).x;
        y3 = cloud->points.at(idx3).y;
        z3 = cloud->points.at(idx3).z;
        // coeffA = cloud->points.at(idx1).y - cloud->points.at(idx2).y;
        // coeffB = cloud->points.at(idx2).x - cloud->points.at(idx1).x;
        // coeffC = (cloud->points.at(idx1).x * cloud->points.at(idx2).y) - (cloud->points.at(idx2).x*cloud->points.at(idx1).y);
        // denom = sqrt((coeffA*coeffA) + (coeffB*coeffB));
        coeffA = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
        coeffB = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
        coeffC = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
        coeffD = -((coeffA*x1) + (coeffB*y1) + (coeffC*z1));
        denom = sqrt((coeffA*coeffA) + (coeffB*coeffB) + (coeffC*coeffC));
        for(int j{0}; j < points_size; j++) {
            distance = fabs((cloud->points.at(j).x * coeffA) + (cloud->points.at(j).y * coeffB) + (cloud->points.at(j).z*coeffC) + coeffD) / denom;
            if(distance < distanceTol)
                buffer.insert(j);
        }
        if(buffer.size() > maxInliers) {
            maxInliers = buffer.size();
            inliersResult = buffer;
        }
        buffer.clear();
    }
	// For max iterations

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
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
