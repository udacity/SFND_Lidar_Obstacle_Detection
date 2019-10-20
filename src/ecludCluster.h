//
// Created by lab on 10/19/19.
//

#ifndef PLAYBACK_ECLDCLUSTER_H
#define PLAYBACK_ECLDCLUSTER_H

#include "kdtree.h"

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed,
        KdTree *tree, float distanceTol);



std::vector<std::vector<int>> eucldCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);



#endif //PLAYBACK_ECLDCLUSTER_H
