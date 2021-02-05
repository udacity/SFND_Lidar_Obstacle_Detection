// PCL lib Functions for processing point clouds
#include "processPointClouds.h"
#include "quiz/ransac/ransac_plane.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // Voxel grid point reduction
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr vgCloud(new pcl::PointCloud<PointT>());
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*vgCloud);

  // Region crop based filtered
  typename pcl::PointCloud<PointT>::Ptr regionCloud(
      new pcl::PointCloud<PointT>());

  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(vgCloud);
  region.filter(*regionCloud);

  // filtering ego car roof points
    typename pcl::PointCloud<PointT>::Ptr roofCloud(
      new pcl::PointCloud<PointT>());

  std::vector<int> roof_indices;
  pcl::CropBox<PointT> roof(true);
  // TODO refactor roof limit as param
  roof.setMin(Eigen::Vector4f(-2., -1.5, -1.0, 1));
  roof.setMax(Eigen::Vector4f(2.0, 1.5, .5, 1));
  roof.setInputCloud(regionCloud);
  roof.filter(roof_indices);

  std::cout << "roof points:" <<roof_indices.size() << std::endl;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(regionCloud);
  extract.setIndices(toPointIndices(roof_indices));
  extract.setNegative(true);
  extract.filter(*regionCloud);



  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return regionCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr planeCloud(
      new pcl::PointCloud<PointT>(*cloud, inliers->indices));
  typename pcl::PointCloud<PointT>::Ptr obstCloud(
      new pcl::PointCloud<PointT>(*cloud));

  // remove the inliers from the obstCloud (originally initialized to the all
  // set)
  auto inliers_positions = inliers->indices;
  sort(inliers_positions.begin(), inliers_positions.end(),
       [](int i, int j) { return j < i; });
  // display_vector<int>(inliers_positions);
  for (auto pos : inliers_positions) {
    obstCloud->erase(obstCloud->begin() + pos);
  }
  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstCloud, planeCloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::PCLSegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // inliers indices
  pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefs);


  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return SeparateClouds(inliers, cloud);
}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // inliers indices

  auto seed_inliers = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
  auto inliers = toPointIndices(std::get<3>(seed_inliers));

  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return SeparateClouds(inliers, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // perform euclidean clustering to group detected obstacles cloud
  std::vector<std::vector<float>> points;
  for (auto point : cloud->points) {
    points.push_back(toVector<PointT>(point));
  }
  KdTree *tree = new KdTree;
  for (int i = 0; i < points.size(); i++) {
    tree->insert(points[i], i);
  }

  for (auto cluster : euclideanCluster(points, tree, clusterTolerance)) {
    typename pcl::PointCloud<PointT>::Ptr clusterCloud(
        new pcl::PointCloud<PointT>());
    for (int index : cluster) {
      clusterCloud->points.push_back(cloud->points[index]);
    }
    // filter-out (discard) too small and too large clusters
    if ((clusterCloud->size() > minSize) && (clusterCloud->size() < maxSize))
      clusters.push_back(clusterCloud);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

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
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}