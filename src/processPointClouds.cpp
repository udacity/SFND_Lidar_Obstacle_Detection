// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <cmath>
#include <unordered_set>

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
    processed[index] = true;

    cluster.push_back(index);
    std::vector<int> nearest = tree->search(points[index], distanceTol);
    for (int i = 0; i < nearest.size(); i++)
    {
        if (!processed[nearest[i]])
            proximity(nearest[i], points, cluster, processed, tree, distanceTol);
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i = 0; i < points.size(); i++)
    {
        if (!processed[i])
        {
            std::vector<int> cluster;
            //auto startTime = std::chrono::steady_clock::now();
            proximity(i, points, cluster, processed, tree, distanceTol);
            //auto endTime = std::chrono::steady_clock::now();
            //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            //std::cout << "proximity took " << elapsedTime.count() << " milliseconds" << std::endl;

            clusters.push_back(cluster);
        }
    }
    return clusters;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object

    typename pcl::PointCloud<PointT>::Ptr vgCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*vgCloud);

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(vgCloud);
    crop.filter(*filteredCloud);

    // removing roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-2, -2, -1, 1));
    roof.setMax(Eigen::Vector4f(5, 4, 2, 1));
    roof.setInputCloud(filteredCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int p : indices)
    {
        inliers->indices.push_back(p);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // int maxIterations = 100;
    // float distanceThreshold = 0.2;
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int i : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[i]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*obstCloud);
    cloud.swap(obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    // Convert to the templated PointCloud
    // ////// my code ///////////////////////////////////////////
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // // // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // // // Optional
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // // // Segment the largest planar component from the remaining cloud
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if (inliers->indices.size() == 0)
    // {
    //     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    //     //break;
    // }

    ////////////////////////////////////////

    float a, b, c, d, distance, x1, y1, z1, x2, y2, z2, x3, y3, z3;
    // outer loop
    for (unsigned i = 0; i < maxIterations; i++)
    {

        // Randomly sample subset and fit line
        //GenerateRandomPointsIndices(cloud, p1, p2, p3);

        std::unordered_set<int> temp;
        while (temp.size() < 3)
        {
            temp.insert(rand() % cloud->points.size());
        }

        auto ptr = std::begin(temp);

        x1 = cloud->points[*ptr].x;
        y1 = cloud->points[*ptr].y;
        z1 = cloud->points[*ptr].z;
        ptr++;
        x2 = cloud->points[*ptr].x;
        y2 = cloud->points[*ptr].y;
        z2 = cloud->points[*ptr].z;
        ptr++;
        x3 = cloud->points[*ptr].x;
        y3 = cloud->points[*ptr].y;
        z3 = cloud->points[*ptr].z;

        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);

        //inner loop
        for (unsigned j = 0; j < cloud->points.size(); j++)
        {
            distance = distance = fabs(a * cloud->points[j].x + b * cloud->points[j].y + c * cloud->points[j].z + d) / sqrt(a * a + b * b + c * c);

            if (distance <= distanceThreshold)
            {
                temp.insert(j);
            }
        }

        if (temp.size() > inliersResult.size())
        {
            inliersResult = temp;
        }
    }

    if (inliersResult.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }

    typename pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int point : inliersResult)
    {
        inliers->indices.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clusterIndices);

    // for (pcl::PointIndices getIndices : clusterIndices)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
    //     for (int index : getIndices.indices)
    //         cloudCluster->points.push_back(cloud->points[index]);

    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;

    //     clusters.push_back(cloudCluster);
    // }

    // // quiz implementation
    // Create kdTree
    KdTree *tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
        tree->insert(points[i], i);
    }

    std::vector<std::vector<int>> clustersIndecies = euclideanCluster(points, tree, clusterTolerance);
    for (std::vector<int> cluster : clustersIndecies)
    {
        // int minSize, int maxSize << use them
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        if (cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            for (int indice : cluster)
                clusterCloud->points.push_back(cloud->points[indice]);

            clusters.push_back(clusterCloud);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    //std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
