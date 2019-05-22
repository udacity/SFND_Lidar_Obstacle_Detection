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

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_croped (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_nonroof (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_roof (new pcl::PointCloud<PointT>);
  std::vector<int> roof_point_ind;

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloud_filtered);

  pcl::CropBox<PointT> box;
  box.setMin(minPoint);
  box.setMax(maxPoint);
  box.setInputCloud(cloud_filtered);
  box.filter(*cloud_croped);

  
  /*
   * TODO:
   * This filter of the roof IT`s NOT working
   * Pls FIX
   */

  pcl::CropBox<PointT> boxcar(true);
  boxcar.setMin(Eigen::Vector4f(-2.5,-1.6,-1,1));
  boxcar.setMax(Eigen::Vector4f(2.8,1.6,0,1));
  boxcar.setInputCloud(cloud_filtered);
  boxcar.filter(roof_point_ind);
  boxcar.filter(*cloud_roof);

  pcl::PointIndices::Ptr roof_indices(new pcl::PointIndices() );


  (*roof_indices).indices = roof_point_ind;

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_croped);
  extract.setIndices(roof_indices);
  extract.setNegative(true);
  extract.filter(*cloud_nonroof);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloud_nonroof;

}


  template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  //

  typename pcl::PointCloud<PointT>::Ptr cloud_obst (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_plane);

  extract.setNegative(true);
  extract.filter(*cloud_obst);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_plane);
  return segResult;
}


  template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }


  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
  return segResult;
}


  template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;

  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for(pcl::PointIndices getIndices: cluster_indices)
  {

    typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
    for(int index : getIndices.indices)
      cloudCluster->points.push_back(cloud->points[index]);
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;
    clusters.push_back(cloudCluster);

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

  template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
  std::unordered_set<int> inliers;
  while(maxIterations--)
  {
	// Randomly sample subset and fit line

    while(inliers.size() < 3)
      inliers.insert(rand()%(cloud->points.size()));

    auto itr = inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    float z1 = cloud->points[*itr].z;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;
    float z2 = cloud->points[*itr].z;
    itr++;
    float x3 = cloud->points[*itr].x;
    float y3 = cloud->points[*itr].y;
    float z3 = cloud->points[*itr].z;
    
	// Measure distance between every point and fitted line

    float a = ((y2 - y1)*(z3 - z1)  - (y3 - y1) * (z2 - z1));
    float b = ((z2 - z1) * (x3 - x1) - (z3 - z1)*(x2 - x1));
    float c = ((x2 - x1) * (y3 - y1) - (x3 - x1)*(y2 - y1));
    float d = (x1 * (y3*z2 - y3*z1 - y1*z2 - y2*z3 + y2*z1 + y1*z3) + 
               y1 * (z3*x2 - z3*x1 - z1*x2 - z2*x3 + z2*x1 + z1*x3) +
               z1 * (x3*y2 - x3*y1 - x1*y2 - x2*y3 + x2*y1 + x1*y3));

    for(int i=0; i < cloud->points.size();i++)
    {
      if(inliers.count(i)>0)
        continue;

      PointT point = cloud->points[i];
      float x0 = point.x;
      float y0 = point.y;
      float z0 = point.z;

      float dist = fabs(a*x0 + b*y0 +c*z0 + d)/sqrt(a*a + b*b + c*c);

      if(dist <= distanceTol)
        inliers.insert(i);
    }

	// If distance is smaller than threshold count it as inlier
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }
  }

	// Return indicies of inliers from fitted line with most inliers
	
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT pointa = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(pointa);
		else
			cloudOutliers->points.push_back(pointa);
	}

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers );

  return segResult;
}

  template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int i, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster,std::vector<bool>& processed, KdTree3d* tree,float distanceTol)
{

  processed[i] = true;
  cluster.push_back(i);


  std::vector<float> p = {
    cloud->points[i].x,
    cloud->points[i].y,
    cloud->points[i].z,
  };
  std::vector<int> nearest = tree->search(p,distanceTol);

  for(int id: nearest)
  {

    if(!processed[id])
      clusterHelper(id,cloud,cluster, processed, tree, distanceTol);
  }
}

  template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree3d* tree, float distanceTol)
{

  std::vector<std::vector<int>> clusters;

  std::vector<bool> processed(cloud->points.size(),false);

  
  int i =0;
  while(i < cloud->points.size())
  {
    if(processed[i])
    {
      i++;
      continue;
    }

    std::vector<int> cluster;
    clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }

  return clusters;

}
