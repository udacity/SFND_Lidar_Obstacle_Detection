#ifndef LINEAR_ALGEBRA_UTILS_H
#define LINEAR_ALGEBRA_UTILS_H

#include <pcl/common/common.h>
#include<vector>

/* Return wether the pointA and pointB are overlapping */
bool overlapping(pcl::PointXYZ pointA, pcl::PointXYZ pointB) {
	return (pointA.x == pointB.x) && (pointA.y == pointB.y) && (pointA.z == pointB.z);

}

/* Compute the unit vector of the AB line */
pcl::PointXYZ vector(pcl::PointXYZ pointA, pcl::PointXYZ pointB) {
	return pcl::PointXYZ((pointB.x - pointA.x), (pointB.y - pointA.y), (pointB.z - pointA.z));
}

float norm(pcl::PointXYZ vect) {
	return (float) sqrt(pow(vect.x, 2) + pow(vect.y, 2) + pow(vect.z, 2));
}

pcl::PointXYZ unit( pcl::PointXYZ vect) {
	auto norm_ = norm(vect);
	return pcl::PointXYZ(vect.x/norm_, vect.y/norm_, vect.z/norm_);
}

float scalar_product(pcl::PointXYZ pointB, pcl::PointXYZ pointA) {
	return (float) ((pointA.x * pointB.x) + (pointA.y * pointB.y) + (pointA.z * pointB.z));
}


pcl::PointXYZ cross_product(pcl::PointXYZ vectA, pcl::PointXYZ vectB) {
    return pcl::PointXYZ((vectA.y * vectB.z) - (vectA.z * vectB.y),
                        (vectA.z * vectB.x) - (vectA.x * vectB.z),
                        (vectA.x * vectB.y) - (vectA.y * vectB.x));
}


pcl::PointXYZ translate(pcl::PointXYZ pointA, pcl::PointXYZ vect) {
    return pcl::PointXYZ(pointA.x + vect.x, pointA.y + vect.y, pointA.z + vect.z);
}


float distance_to_line(pcl::PointXYZ point, pcl::PointXYZ linePointA, pcl::PointXYZ vectorAB) {
	auto vectorAX = vector(linePointA, point);
	return sqrt(1-pow(scalar_product(unit(vectorAX), unit(vectorAB)),2)) * norm(vectorAX);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr addZData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int pointsNbr=100)
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

#endif // LINEAR_ALGEBRA_UTILS_H