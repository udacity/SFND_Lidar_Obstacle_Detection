// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint)
{
    std::cout << "Input point size: " << cloud->points.size() << std::endl;

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr output_cloud (new pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloud_filtered);
    
    pcl::CropBox<PointT> region;
    region.setInputCloud(cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setNegative(false);
    region.filter(*cloud_region);

    pcl::CropBox<PointT> crop_car_box;
    std::vector<int> indicies;
    crop_car_box.setMin(Eigen::Vector4f(-1.5, -1.5, -1.0, 1.));
    crop_car_box.setMax(Eigen::Vector4f(2.5, 1.7, -0.4, 1));
    crop_car_box.setInputCloud(cloud_region);
    crop_car_box.setNegative(true);
    crop_car_box.filter(indicies);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto index : indicies)
        inliers->indices.push_back(index);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.filter(*output_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "filtered size: " << cloud_filtered->points.size() << std::endl;
    return output_cloud;
    
}

inline int get_random_int(const int max_val)
{
	return int(max_val * ((double) rand() / RAND_MAX));
}

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
		PointT point1 = cloud->points[idx1];
		PointT point2 = cloud->points[idx2];
		PointT point3 = cloud->points[idx3];

		A = ((point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y));
		B = ((point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z));
		C = ((point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x));
		D = -(point1.x * A + B * point1.y + C * point1.z);

		// Measure distance between every point and fitted line
		int num_inliers = 0;
		for (int index = 0; index < cloud->points.size(); index++) {
			PointT point = cloud->points[index];
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
		PointT point = cloud->points[index];
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles_p (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane_p (new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    // extract the plane pointclouds
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_p);

    // extract the obstacle pointclout
    extract.setNegative(true);
    extract.filter(*obstacles_p);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_p, plane_p);
    return segResult;
}


// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneOld
// (typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
// 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

//     // TODO:: Fill in this function to find inliers for the cloud.
//     pcl::SACSegmentation<PointT> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setMethodType(pcl::SACMODEL_PLANE);
//     seg.setModelType(pcl::SAC_RANSAC);
//     seg.setMaxIterations(maxIterations);
//     seg.setDistanceThreshold(distanceThreshold);

//     seg.setInputCloud(cloud);
//     seg.segment(*inliers, *coefficients);
//     if (inliers->indices.size() == 0) {
//         std::cerr << "Could not segment the plane indicies" << std::endl;
//     }
//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
//     return segResult;
// }

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane
(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr obstacles_p (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane_p (new pcl::PointCloud<PointT>);

    for(int index = 0; index < cloud->points.size(); index++) {
 		PointT point = cloud->points[index];
		if(inliers.count(index)) {
            plane_p->points.push_back(cloud->points[index]);
        }
        else {
            obstacles_p->points.push_back(cloud->points[index]);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_p, plane_p);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to 
    // group detected obstacles create the kd tree with the point cloud
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
 
    // Initialize Eucledian filtering code and its relevant parameters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    // Setup appropriate clustering sizes for different filters
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // for each point in the point cloud cluster indicies
    auto count = 0;
    for (auto it=cluster_indices.begin(); it != cluster_indices.end(); it++) {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
        for(auto pit=it->indices.begin(); pit != it->indices.end(); pit++) {
            cluster_cloud->points.push_back(cloud->points[*pit]);
        }
        std::cout << "Cluster id: " << count << ", size: " << cluster_cloud->points.size() << std::endl;
        clusters.push_back(cluster_cloud);
        count ++;
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}