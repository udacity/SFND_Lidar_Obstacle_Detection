// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create new output point cloud
    typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>);

    // Create VoxelGrid filter to downsample PCD, using grid resolution equal to filterRes
    typename pcl::VoxelGrid<PointT> vGrid;
    vGrid.setInputCloud(cloud);
    vGrid.setLeafSize(filterRes, filterRes, filterRes);
    vGrid.filter(*filterCloud);

    // Crop points outside region of interest using CropBox
    typename pcl::CropBox<PointT> crop;
    crop.setInputCloud(filterCloud);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*filterCloud);

    // Crop points on roof using CropBox
    crop.setNegative(true) ;
    crop.setInputCloud(filterCloud);
    crop.setMin(Eigen::Vector4f(-1.6, -1.7, -3., 1));
    crop.setMax(Eigen::Vector4f( 2.6,  1.7,  3., 1));
    crop.filter(*filterCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filterCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>), planeCloud(new pcl::PointCloud<PointT>);
    // Declare the output clouds (plane, obstacle)

    // First create the filtering object
    typename pcl::ExtractIndices<PointT> extract;
    // Extract the inliers and assign them to the plane point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);
    // Alternative: set up plane cloud to comprise of points in the original cloud at the inliers indices
    // for (int index : inliers->indices)
    //     planeCloud->points.push_back(cloud->points[index])

    // Extract the outliers (by settinge negative = true) and assign them to the obstacle point cloud
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // Allocate indices for the inliers (points belonging to the plane)
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};

    // Allocate model coefficients 
    pcl::ModelCoefficients::Ptr coeffs {new pcl::ModelCoefficients()};
    // Create the segmentation object
    typename pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Set model to plane and segmentation method to RANSAC
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    // Distance tolerance to consider points inside plane
    seg.setDistanceThreshold(distanceThreshold);
    
    // Set our cloud as input to the segmentation object
    seg.setInputCloud(cloud);
    // Segment the largest planar component from the input cloud
    // The resulting indices are assigned to the inliers: 
    // Notice how we de-reference the pointer before passing it to the segment method
    // The coefficients of the resulting plane will be stored in coeffs (also dereferenced)
    seg.segment(*inliers, *coeffs);
        
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction. The input will be our obstacles cloud
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    tree->setInputCloud (cloud);
    // Create a vector of PointIndices which contain the indices of each detected cluster
    std::vector<pcl::PointIndices> cluster_indices;
    // Create EuclideanClusterExtraction object with template point type PointT. Use cluster_indices vector to put the indices of each obstacle
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // distance tolerance --> any points within will be grouped together
    ec.setMinClusterSize (minSize); // min number of points in cluster
    ec.setMaxClusterSize (maxSize); // max number of points in cluster
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (auto cluster_idx : cluster_indices)
    {
        // Create cluster by selecting indices from one of the elements in cluster_indices
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>());
        for (int idx : cluster_idx.indices)
            cluster->points.push_back(cloud->points[idx]);
        // Set cluster width and height
        cluster->width = cluster->size ();
        cluster->height = 1;
        cluster->is_dense = true;
        
        // std::cout << "PointCloud representing the Cluster: " << cluster->size () << " data points." << std::endl;

        // Insert in output cluster vector
        clusters.push_back(cluster);
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