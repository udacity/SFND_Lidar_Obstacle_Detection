# Implementing KD-Tree and Euclidean Clustering

A KD-Tree is a binary tree that splits points between alternating axes. By seperating space by splitting regions, it can make it much faster to do nearest neighbor search when using an algorithm like euclidean clustering. In this quiz we will be looking at a 2D example, so the the tree will be a 2D-Tree. In the first part of the quiz you will be working from `src/quiz/cluster/kdtree.h` and filling in the function `insert` which take a 2D point represented by a vector of 2 floats, and a point ID, this is just a way to uniquely identify points and a way to tell which index they are from the overall point cloud. To complete the `insert` function let's first talk about how a KD-Tree splits information.

## Inserting Points into the Tree

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/2dpoints.png" width="700" height="400" />

The image above shows what the 2d points look like, in this simple example there is only 11 points, and there is 3 clusters where points are in close proximity to each other that you will be finding. First you want to create your tree, in `src/quiz/cluster/cluster.cpp` there is a function for rendering the tree after points have been inserted into it, this rendering shows line seperations, blue lines splitting x regions and red lines splitting y regions. The image below shows what the tree looks like after all 11 points have been inserted.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/kdtree.png" width="700" height="400" />

Now lets talk about how exactly the tree is created. At the very beginning when the tree is empty, root is NULL, the point inserted becomes the root, and splits the x region. Here is what this visually looks like, after inserting the first point (-6.2, 7).

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/kdtree1.png" width="700" height="400" />

The next point is (-6.3,8.4), since -6.3 is less than -6.2 this Node will be created and be apart of root's left node, and now the point (-6.3,8.4) will split the region in the y dimension. The root was at depth 0, and split the x region, the next point become the left child of root and had a depth of 1, and split the y region. A point at depth 2 will split the x region again, so the split can actually be calculated as depth % 2, where 2 is the number of dimensions we are working with. The image below show how the tree looks after inserting the second point.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/kdtree2.png" width="700" height="400" />

Then here is what the tree looks like after inserting two more points (-5.2,7.1), (-5.7,6.3), and having another x split division from point (-5.7,6.3) being at depth 2 in the tree.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/kdtree4.png" width="700" height="400" />

The image below shows so far what the tree looks like after inserting those 4 points. The labeled nodes A, B, C, D, and E are all NULL but if the next point (7.2,6.1) is inserted, whill of those 5 nodes will it be assigned to ?

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/kdtree5.png" width="700" height="400" />

The answer is D. Let's look at why this is. First the root (-6.2, 7) and the point(7.2, 6.1) x region will be compared. 7.2 is greater than -6.2 so the new point will branch off to the right to (-5.2, 7.1). Next the y region will be compared, 6.1 is less than 7.1 so the new point will branch off to the left to (-5.7,6.3). Last the x region will be compared again, 7.2 is greater than -5.7 so the new point will branch to the right and will be Node D.

Logically this is how points are inserted, how about doing this in C++? Implementing a recursive helper function to insert points can be a very nice way to update Nodes. The basic idea is the tree is traversed until the Node is arrives at is NULL in which case a new Node is created and assigned at that current NULL Node. For assinging a Node, one idea is to use a double pointer, you could pass in a pointer to the node, starting at root, and then when you want to assign that node, derefrence pointer and assign it to the newly created Node. Another way of achieving this is by using a pointer reference as well. 

### Improving the Tree Structure

Having a balanced tree that evenly splits regions improves the search time for finding points later. To improve the tree insert points that alternate between splitting the x region and the y region. To do this pick the median of sorted x and y points. For instance if you are inserting the first four points that we used above (-6.3,8.4),(-6.2,7),(-5.2,7.1),(-5.7,6.3) we would first insert (-5.2,7.1) since its the median for the x points, if there is an even number of elements the lower median is chosen. The next point to be insorted would be (-6.2,7) the median of the three points for y. Followed by (-5.7,6.3) the lower median between the two for x, and then finally (-6.3,8.4). This ordering will allow the tree to more evenly split the region space and improving searching later.

## Searching Nearby Points in the Tree

Once points are able to be inserted into the tree, the next step is being able to search for nearby points (points within a distance of distanceTol) inside the tree compared to a given pivot point. The kd-tree is able to split regions and allows certain regions to be completly ruled out, speeding up the process of finding nearby neighbors. The naive approach of finding nearby neighbors is to go through every single point in the tree and compare its distance with the pivots, selecting point indices that fall with in the distance tolerance. Instead with the kd-tree we can compare distance within a boxed square that is 2 x distanceTol for length, centered around the pivot point. If the current node point is within this box then we can directly calculate the distance and see if we add it to the list of `ids`. Then we see if our box crosses over the node division region and if it does compare that next node. We do this recursively, with the advantage being that if the box region is not inside some division region we completly skip that branch.

### Exercise 

Fill in the `kdtree.h` search function.


## Euclidean Clustering

Once the kd-tree method for searching for nearby points is implemented, its not difficult to implement a euclidean clustering method that groups indidual cluster indicies based on their proximity. Inside `cluster.cpp` there is a function called `euclideanCluster` which returns a vector of vector ints, this is the list of each cluster's indices. To perform the clustering, iterate through each point in the cloud and keep track of which points have been processed already. For each point add it to a cluster group then get a list of all the points in proximity to that point. For each point in proximity if it hasn't already been processed add it to the cluster group and repeat the process of calling proximity points. Once the recursion stops for the first cluster group, create a new cluster and move through the point list. Once all the points have been processed there will be a certain number of cluster groups found.

### Exercise

```
list of clusters 

Proximity(point,cluster)
	If point has not been processed
		mark point as processed
		add point to cluster
		nearby points = tree(point)
		Iterate through each nearby point
			Proximity(cluster)


Iterate through each point
	Create cluster
	Proximity(point,cluster)
	cluster add cluster

return clusters

```

When running the euclidean clustering function on the data here is the results

```
euclideanCluster(points, tree, 3.0);

```

There are three clusters found, using a distance tolerance of 3.0. Each cluster group is colored a differently, red, green, and blue.

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/clusterKdtree.png" width="700" height="400" />


