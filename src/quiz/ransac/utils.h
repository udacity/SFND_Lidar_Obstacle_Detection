/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @desc linear algebra/vector utils + other
 */

#ifndef LINEAR_ALGEBRA_UTILS_H
#define LINEAR_ALGEBRA_UTILS_H

#include <pcl/common/common.h>
#include<vector>
#include<unordered_set>

/* Return wether the pointA and pointB are overlapping */
template <typename PointT>
bool overlapping(PointT pointA, PointT pointB) {
	return (pointA.x == pointB.x) && (pointA.y == pointB.y) && (pointA.z == pointB.z);

}

/* Compute the unit vector of the AB line */
template <typename PointT1, typename PointT2>
pcl::PointXYZ vector(PointT1 pointA, PointT2 pointB) {
	return pcl::PointXYZ((pointB.x - pointA.x), (pointB.y - pointA.y), (pointB.z - pointA.z));
}

template <typename PointT>
float norm(PointT vect) {
	return (float) sqrt(pow(vect.x, 2) + pow(vect.y, 2) + pow(vect.z, 2));
}

template <typename PointT>
pcl::PointXYZ unit( PointT vect) {
	auto norm_ = norm(vect);
	return PointT(vect.x/norm_, vect.y/norm_, vect.z/norm_);
}

template <typename PointT1, typename PointT2>
float scalar_product(PointT1 pointB, PointT2 pointA) {
	return (float) ((pointA.x * pointB.x) + (pointA.y * pointB.y) + (pointA.z * pointB.z));
}


template <typename PointT1, typename PointT2>
pcl::PointXYZ cross_product(PointT1 vectA, PointT2 vectB) {
    return pcl::PointXYZ((vectA.y * vectB.z) - (vectA.z * vectB.y),
                        (vectA.z * vectB.x) - (vectA.x * vectB.z),
                        (vectA.x * vectB.y) - (vectA.y * vectB.x));
}

template <typename PointT>
pcl::PointXYZ translate(PointT pointA, pcl::PointXYZ vect) {
    return pcl::PointXYZ(pointA.x + vect.x, pointA.y + vect.y, pointA.z + vect.z);
}

template <typename PointT>
float distance_to_line(PointT point, PointT linePointA, PointT vectorAB) {
	auto vectorAX = vector(linePointA, point);
	return sqrt(1-pow(scalar_product(unit(vectorAX), unit(vectorAB)),2)) * norm(vectorAX);
}

typename pcl::PointCloud<pcl::PointXYZ>::Ptr addZData(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int pointsNbr=100)
{
    // Add outliers
    auto points = pointsNbr ;
    while(points--){
        double rz = 2*(((double) rand() / (RAND_MAX))-0.5);
        cloud->points.push_back(pcl::PointXYZ(0,0,5*rz));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}

template <typename PointT>
std::vector<float> toVector(PointT point) {
    return {point.x, point.y, point.z};
}

pcl::PointIndices::Ptr toPointIndices(std::vector<int> indices) {
    pcl::PointIndices::Ptr pointIndices {new pcl::PointIndices};
    for (int index: indices){
        pointIndices->indices.push_back(index);
    }
    return pointIndices;
}

pcl::PointIndices::Ptr toPointIndices(std::unordered_set<int> indices) {
    pcl::PointIndices::Ptr pointIndices {new pcl::PointIndices};
    for (int index: indices){
        pointIndices->indices.push_back(index);
    }
    return pointIndices;
}

#endif // LINEAR_ALGEBRA_UTILS_H