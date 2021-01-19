// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <unordered_set>
//#include "quiz/cluster/kdtree.h"

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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_crop_filtered (new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> pcd_cropper(true);
    pcd_cropper.setMin(minPoint);
    pcd_cropper.setMax(maxPoint);
    pcd_cropper.setInputCloud(cloud_filtered);
//    pcd_cropper.setNegative(true);
    pcd_cropper.filter(*cloud_crop_filtered);

    pcl::CropBox<PointT> roof(true);
    std::vector<int> roof_indices;
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_crop_filtered);
    roof.filter(roof_indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    for(auto ind: roof_indices)
        inliers->indices.push_back(ind);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_crop_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_crop_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>), roadCloud (new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*roadCloud);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
//	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations(maxIterations);
//    seg.setDistanceThreshold (distanceThreshold);
//
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);
//
//    if (inliers->indices.size () == 0)
//    {
//        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//    }

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
        z2 = cloud->points.at(idx2).z;
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
            if(distance < distanceThreshold)
                buffer.insert(j);
        }
        if(buffer.size() > maxInliers) {
            maxInliers = buffer.size();
            inliersResult = buffer;
        }
        buffer.clear();
    }
    std::vector<int> inlierVec(inliersResult.begin(), inliersResult.end());
    inliers->indices = inlierVec;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int index, std::vector<bool> &proc_status, KdTree &tree, float distanceTol, std::vector<int> &cluster)
{
    cluster.push_back(index);
    proc_status.at(index) = true;
    std::vector<int> nearby_points = tree.search(cloud->points.at(index), distanceTol);
    for(auto np : nearby_points)
    {
        if(!proc_status.at(np))
            Proximity(cloud, np, proc_status, tree, distanceTol, cluster);
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<int> cluster;
    std::vector<bool> proc_status(cloud->points.size(), false);
    for(int i {0}; i < cloud->points.size(); i++)
    {
        if(!proc_status.at(i))
        {
            Proximity(cloud, i, proc_status, *tree, distanceTol, cluster);
            clusters.push_back(cluster);
            cluster.clear();
        }
    }
    return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction

//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    tree->setInputCloud (cloud);
//
    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance (clusterTolerance); // 2cm
//    ec.setMinClusterSize (minSize);
//    ec.setMaxClusterSize (maxSize);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud);
//    ec.extract (cluster_indices);

    KdTree* tree = new KdTree;
    tree->dim = 3;
    for (int i=0; i < cloud->points.size(); i++)
        tree->insert(cloud->points[i],i);
    std::vector<std::vector<int>> cluster_vec = euclideanCluster(cloud, tree, clusterTolerance);
    for(auto clus: cluster_vec)
    {
        pcl::PointIndices pInd;
        pInd.indices = clus;
        if(clus.size() > minSize && clus.size() < maxSize)
            cluster_indices.push_back(pInd);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        typename pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            tempCloud->points.push_back (cloud->points[*pit]);
        tempCloud->width = tempCloud->points.size ();
        tempCloud->height = 1;
        tempCloud->is_dense = true;
        clusters.push_back(tempCloud);
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