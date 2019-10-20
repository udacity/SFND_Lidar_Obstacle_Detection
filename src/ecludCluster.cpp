//
// Created by lab on 10/19/19.
//

#include "ecludCluster.h"

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol){
    processed[indice]=true;
    cluster.push_back(indice);
    std::vector<int> nearest = tree->search(points[indice], distanceTol);
    for(int i: nearest){
        if(processed[i]){
            clusterHelper( i, points, cluster, processed, tree, distanceTol);
        }
    }
}


std::vector<std::vector<int>> eucldCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster
    int pointCount = points.size();
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(pointCount, false);

    int i=0;
    while(i < pointCount) {
        if(processed[i]){
            i++;
            continue;
        }
        std::vector<int> cluster;
        clusterHelper( i, points, cluster, processed, tree, distanceTol);
        i++;
    }

    return clusters;
}