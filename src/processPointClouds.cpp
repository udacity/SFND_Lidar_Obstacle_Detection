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

extern int use_pcl_ransac;

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
bool get_plane_coefficients(
    std::vector<Eigen::Vector3f>& points,
    Eigen::Vector4f &model_coefficients)
{
    Eigen::Vector3f vec10 = (points[1] - points[0]);
    Eigen::Vector3f vec20 = (points[2] - points[0]);

    model_coefficients[0] = (vec10[1] * vec20[2] - vec10[2] * vec20[1]);
    model_coefficients[1] = (vec10[2] * vec20[0] - vec10[0] * vec20[2]);
    model_coefficients[2] = (vec10[0] * vec20[1] - vec10[1] * vec20[0]);
    model_coefficients[3] = 0;

    Eigen::Vector4f ref_point(points[0][0], points[0][1], points[0][2], 0);
    model_coefficients[3] = -1 * (model_coefficients.dot(ref_point));
    return true;
}

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	srand(0);

	// For max iterations 
	auto iteration_count = 0;
	int max_inliers = 0;
	std::vector<int> best_line(4);
    std::vector<Eigen::Vector3f> points;
    points.resize(3);
    Eigen::Vector4f coefficients;
    Eigen::Vector4f best_plane_coefficients;

	while (iteration_count < maxIterations) {	

		// generate 3 random points for ransac plane fitting
        for (int i=0; i < 3; i++) {
	    	int idx = get_random_int(cloud->points.size());
    		PointT point = cloud->points[idx];
            points[i] = Eigen::Vector3f(point.x, point.y, point.z);
        }

        bool success = get_plane_coefficients<PointT>(points, coefficients);
        if (not success)
            continue;

		// Measure distance between every point and fitted line
		int num_inliers = 0;
        double abs_normal = sqrt(coefficients[0] * coefficients[0] +
                                coefficients[1] * coefficients[1] +
                                coefficients[2] * coefficients[2]);
		for (int index = 0; index < cloud->points.size(); index++) {
            PointT point = cloud->points[index];
            float distance = fabs(coefficients[0]*point.x +
                                coefficients[1]*point.y + 
                                coefficients[2]*point.z + 
                                coefficients[3]) / abs_normal;
	     	// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) {
				num_inliers += 1;
			}
		}

		// best inlier function
		if (num_inliers > max_inliers) {
			max_inliers = num_inliers;
            best_plane_coefficients = coefficients;
		}
		iteration_count += 1;
	}

    double abs_normal = sqrt(best_plane_coefficients[0] * best_plane_coefficients[0] +
                            best_plane_coefficients[1] * best_plane_coefficients[1] +
                            best_plane_coefficients[2] * best_plane_coefficients[2]);
	for (int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        float distance = fabs(best_plane_coefficients[0]*point.x + 
                              best_plane_coefficients[1]*point.y + 
                              best_plane_coefficients[2]*point.z + 
                              best_plane_coefficients[3]) / abs_normal;
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


template<typename PointT>
std::unordered_set<int> RansacPCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceTol);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not segment the plane indicies" << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::unordered_set<int>  inliersResult;
;
    for (auto idx: inliers->indices) {
        inliersResult.insert(idx);
    }     
    return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane
(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliers;
    inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);
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