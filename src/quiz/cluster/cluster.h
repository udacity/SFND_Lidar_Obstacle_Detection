/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @create date 2021-02-03 10:36:31
 * @modify date 2021-02-03 10:36:31
 * @desc euclidean clustering implementation
 */

#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include <vector>
#include "kdtree.h"


void proximity(int index, const std::vector<std::vector<float>> &points,
               std::vector<bool> &processed, KdTree *tree, float distanceTol,
               std::vector<int> &cluster) {
  if (!processed[index]) {
    processed[index] = true;
    cluster.push_back(index);
    auto neighbors = tree->search(points[index], distanceTol);
    for (auto neighbor_index : neighbors) {
      proximity(neighbor_index, points, processed, tree, distanceTol, cluster);
    }
  }
}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree,
                 float distanceTol) {
  std::vector<bool> processed(points.size(), false);
  std::vector<std::vector<int>> clusters;

  for (int index = 0; index < points.size(); index++) {
    if (!processed[index]) {
      std::vector<int> cluster;
      proximity(index, points, processed, tree, distanceTol, cluster);
      clusters.push_back(cluster);
    }
  }
  return clusters;
}

#endif
