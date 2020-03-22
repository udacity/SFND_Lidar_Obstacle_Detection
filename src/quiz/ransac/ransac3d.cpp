/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time Ransac process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // Loop max iterations
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        // Randomly select 3 unique indicies
        while (inliers.size() < 3)
            inliers.insert(rand() % cloud->points.size());

        // Assign values
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        ++itr;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        ++itr;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Determine plane coefficients
        float a, b, c, d;
        a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        d = -(a * x1 + b * y1 + c * z1);

        // Measure distance between every point and fitted plane
        for (size_t index = 0; index < cloud->size(); ++index)
        {
            if (inliers.count(index))
                continue;
            
            // point
            pcl::PointXYZ point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // Orthogonal distance
            float distance;
            distance = std::fabs(a * x4 + b * y4 + c * z4 + d) / std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol)
            {
                inliers.insert(index);
            }
        }
        // Swap the indices if greater than past count.
        if (inliers.size() > inliersResult.size())
        {
            inliersResult.swap(inliers);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}

int main()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    std::unordered_set<int> inliers = Ransac(cloud, 10, 0.25);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
