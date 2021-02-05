/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @desc Ransac for plane detection
 */

#ifndef RANSAC_PLANE_H
#define RANSAC_PLANE_H

#include "utils.h"

#include <pcl/common/common.h>

#include <cstdlib>
#include <math.h>
#include <tuple>
#include <unordered_set>


template<typename PointT>
std::tuple<int, int, int, std::unordered_set<int>>
RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
       float distanceTol) {

  std::tuple<int, int, int, std::unordered_set<int>> results;
  srand(time(NULL));
  if (cloud->size() > 2) {
    for (int iter = 0; iter < maxIterations; iter++) {
      std::unordered_set<int> inliers;

      // Randomly sample subset (3 points) to define the plane
      auto pointAIndex = rand() % cloud->size();
      auto pointBIndex = rand() % cloud->size();
      auto pointCIndex = rand() % cloud->size();

      // Measure distance between every point and fitted line
      auto pointA = cloud->at(pointAIndex);
      auto pointB = cloud->at(pointBIndex);
      auto pointC = cloud->at(pointCIndex);

      auto vectorAB = vector<PointT>(pointA, pointB);
      auto vectorAC = vector<PointT>(pointA, pointC);
      auto ABxAC = cross_product<pcl::PointXYZ, pcl::PointXYZ>(vectorAB, vectorAC);

      while (norm(ABxAC) == 0) {
        // should avoid collinearity
        pointBIndex = rand() % cloud->size();
        pointCIndex = rand() % cloud->size();

        pointB = cloud->at(pointBIndex);
        pointC = cloud->at(pointCIndex);

        vectorAB = vector<PointT>(pointA, pointB);
        vectorAC = vector<PointT>(pointA, pointC);
        ABxAC = cross_product<pcl::PointXYZ, pcl::PointXYZ>(vectorAB, vectorAC);
      }

      //  Insert selected seed points (pointA and pointB) indices as inliers
      inliers.insert(pointAIndex);
      inliers.insert(pointBIndex);
      inliers.insert(pointCIndex);

      for (int index = 0; index < cloud->points.size(); index++) {
        // If distance is smaller than threshold count it as inlier
        if (inliers.count(index) > 0)
          continue; // it is one the points A, B defining the line
                    // Measure distance between every point and fitted line
                    // If distance is smaller than threshold count it as inlier
        auto distance =
            fabs(scalar_product<pcl::PointXYZ, pcl::PointXYZ>(unit<pcl::PointXYZ>(ABxAC), vector<PointT>(pointA, cloud->at(index))));
        if (distance < distanceTol) {
          inliers.insert(index);
        }
      }

      if (inliers.size() > std::get<3>(results).size()) {
        results = std::tie(pointAIndex, pointBIndex, pointCIndex, inliers);
        std::cout << "cross prod. AB x AB (should be 0)"
                  << cross_product<pcl::PointXYZ>(vectorAB, vectorAB) << std::endl;
        std::cout << "cross prod. AB x AC "
                  << unit<pcl::PointXYZ>(cross_product<pcl::PointXYZ>(vectorAB, vectorAC)) << std::endl;
        std::cout << "inliers size ->" << inliers.size() << std::endl;
      }
    }
  }
  // Return indicies of inliers from fitted plane with most inliers
  return results;
}

#endif // RANSAC_PLANE_H
