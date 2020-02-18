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

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	auto iteration_count = 0;
	int max_inliers = 0;
	std::vector<int> best_line(4);
	float A, B, C, D;
	while (iteration_count < maxIterations) {
		// Randomly sample subset and fit line)
		int idx1 = get_random_int(cloud->points.size());
		int idx2 = get_random_int(cloud->points.size());
		int idx3 = get_random_int(cloud->points.size());
		
		// same number has been generated, ignore and continue
		if ((idx1 == idx2) || (idx1 == idx3) || (idx2 == idx3))
		    continue;

		// points here
		pcl::PointXYZ point1 = cloud->points[idx1];
		pcl::PointXYZ point2 = cloud->points[idx2];
		pcl::PointXYZ point3 = cloud->points[idx3];

		A = ((point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y));
		B = ((point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z));
		C = ((point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x));
		D = -(point1.x * A + B * point1.y + C * point1.z);

		// Measure distance between every point and fitted line
		int num_inliers = 0;
		for (int index = 0; index < cloud->points.size(); index++) {
			pcl::PointXYZ point = cloud->points[index];
			float distance = fabs(A*point.x + B*point.y + C*point.z + D) / (sqrt(A*A + B*B + C*C));
	     	// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) {
				num_inliers += 1;
			}
		}
		//printf("Iteration: %d, inliers: %d", iteration_count, num_inliers);

		// best inlier function
		if (num_inliers > max_inliers) {
			//printf("A=%f, B=%f, C=%f, num_inliers=%d, Best: %d\n", A, B, C, num_inliers, max_inliers);
			max_inliers = num_inliers;
			best_line[0] = A;
			best_line[1] = B;
			best_line[2] = C;
			best_line[3] = D;
		}
		iteration_count += 1;
	}

	// Return indicies of inliers from fitted line with most inliers
	A = best_line[0];
	B = best_line[1];
	C = best_line[2];
	D = best_line[3];
	for (int index = 0; index < cloud->points.size(); index++) {
		pcl::PointXYZ point = cloud->points[index];
		float distance = fabs(A*point.x + B*point.y + C*point.z + D) / (sqrt(A*A + B*B + C*C));
		// If distance is smaller than threshold count it as inlier
		if (distance < distanceTol) {
			inliersResult.insert(index);
		}
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	auto iteration_count = 0;
	int max_inliers = 0;
	pcl::PointXYZ best_line;
	while (iteration_count < maxIterations) {
		// Randomly sample subset and fit line)
		int idx1 = get_random_int(cloud->points.size());
		int idx2 = get_random_int(cloud->points.size());
		printf("Index: %d, %d\n", idx1, idx2);
		
		// same number has been generated, ignore and continue
		if (idx1 == idx2)
		    continue;

		pcl::PointXYZ point1 = cloud->points[idx1];
		pcl::PointXYZ point2 = cloud->points[idx2];

		float A = point1.y - point2.y;
		float B = point2.x - point1.x;
		float C = (point1.x) * (point2.y) - (point2.x) * (point1.y);
		// Measure distance between every point and fitted line
		int num_inliers = 0;
		for (int index = 0; index < cloud->points.size(); index++) {
			pcl::PointXYZ point = cloud->points[index];
			float distance = fabs(A*point.x + B*point.y + C) / (sqrt(A*A + B*B));
	     	// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) {
				num_inliers += 1;
			}
		}

		printf("Iteration: %d, inliers: %d", iteration_count, num_inliers);

		// best inlier function
		if (num_inliers > max_inliers) {
			printf("A=%f, B=%f, C=%f, num_inliers=%d, Best: %d\n", A, B, C, num_inliers, max_inliers);
			max_inliers = num_inliers;
			best_line.x = A;
			best_line.y = B;
			best_line.z = C;
		}
		iteration_count += 1;
	}

	// Return indicies of inliers from fitted line with most inliers
	float A = best_line.x;
	float B = best_line.y;
	float C = best_line.z;
	for (int index = 0; index < cloud->points.size(); index++) {
		pcl::PointXYZ point = cloud->points[index];
		float distance = fabs(A*point.x + B*point.y + C) / (sqrt(A*A + B*B));
		// If distance is smaller than threshold count it as inlier
		if (distance < distanceTol) {
			inliersResult.insert(index);
		}
	}
	
	return inliersResult;
}


void Ransac_PCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	std::unordered_set<int> inliersResult;
	auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPCL3D took " << elapsedTime.count() << " milliseconds" << std::endl;
}

int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	int maxIterations = 100;
	float distanceThreshold = 0.1;
	std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);
	Ransac_PCL(cloud, maxIterations, distanceThreshold);

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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,0,1));
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
