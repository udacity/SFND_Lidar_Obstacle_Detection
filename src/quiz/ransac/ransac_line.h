/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @desc Ransac for line detection (generalized 3d)
 */
#ifndef RANSAC_LINE_H
#define RANSAC_LINE_H

#include "utils.h"

#include <pcl/common/common.h>

#include <cstdlib>
#include <math.h>
#include <tuple>
#include <unordered_set>

std::tuple<int, int, std::unordered_set<int>>
RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations,
       float distanceTol) {

  // Small sanity checks (to move to tests)
  std::cout << "S. check "
            << scalar_product(
                   unit(vector(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1, 0, 0))),
                   unit(vector(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(2, 0, 0))))
            << std::endl;
  std::cout << "S. check norm"
            << norm(
                   unit(vector(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(3, 0, 0))))
            << std::endl;

  std::tuple<int, int, std::unordered_set<int>> results;

  srand(time(NULL));
  if (cloud->size() > 1) {
    // TODO distinguish the case with only 2 points
    for (int iter = 0; iter < maxIterations; iter++) {
      std::unordered_set<int> inliers;

      // Randomly sample subset (2 points) and fit line
      auto pointAIndex = rand() % cloud->size();
      auto pointA = cloud->at(pointAIndex);
      auto pointBIndex = rand() % cloud->size();
      auto pointB = cloud->at(pointBIndex);

      while ((pointAIndex == pointBIndex) || overlapping(pointA, pointB)) {
        // ensure 2 distinct points (by content not by index) re-draw point B
        pointBIndex = rand() % cloud->size();
        pointB = cloud->at(pointBIndex);
        // std::cout << "tie resolved" << std::endl;
      }
      //  Insert selected seed points (pointA and pointB) indices as inliers
      inliers.insert(pointAIndex);
      inliers.insert(pointBIndex);

      // std::cout << "line (points): " << pointA << "," << pointB << std::endl;

      // Measure distance between every point and fitted line
      auto vectorAB = vector(pointA, pointB);

      // std::cout << "Distance B to AB (should be 0)" <<
      // distance_to_line(pointB, pointA, vectorAB) << std::endl;
      for (int index = 0; index < cloud->points.size(); index++) {
        // If distance is smaller than threshold count it as inlier
        if (inliers.count(index) > 0)
          continue; // it is one the points A, B defining the line
        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        if (distance_to_line(cloud->at(index), pointA, vectorAB) <
            distanceTol) {
          inliers.insert(index);
        }
      }

      if (inliers.size() > std::get<2>(results).size())
        results = std::tie(pointAIndex, pointBIndex, inliers);
    }
  }
  // Return indicies of inliers from fitted line with most inliers
  return results;
}

#endif // RANSAC_LINE_H