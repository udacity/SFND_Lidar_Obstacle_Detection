/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
    int dim = 2;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node &node, Node **refNode, uint level) {
        uint axis {level % dim};
        if(node.point.at(axis) < (*refNode)->point.at(axis)) {
            if((*refNode)->left == NULL)
                (*refNode)->left = &node;
            else
                insertHelper(node, &((*refNode)->left), ++level);
        } else {
            if((*refNode)->right == NULL)
                (*refNode)->right = &node;
            else
                insertHelper(node, &((*refNode)->right), ++level);
        }
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        Node* new_point = new Node(point, id);
        if(root == NULL)
            root = new_point;
        else {
            insertHelper(*new_point, &root, 0);
        }

	}

    void insert(pcl::PointXYZ point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        std::vector<float> float_point;
        float_point.push_back(point.x);
        float_point.push_back(point.y);
        float_point.push_back(point.z);
        Node* new_point = new Node(float_point, id);
        if(root == NULL)
            root = new_point;
        else {
            insertHelper(*new_point, &root, 0);
        }

    }

    void insert(pcl::PointXYZI point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        std::vector<float> float_point;
        float_point.push_back(point.x);
        float_point.push_back(point.y);
        float_point.push_back(point.z);
        Node* new_point = new Node(float_point, id);
        if(root == NULL)
            root = new_point;
        else {
            insertHelper(*new_point, &root, 0);
        }

    }

	void searchHelper(std::vector<float> box, Node** node, std::vector<int> &ids, uint level, std::vector<float> target, float distanceTol)
    {
	    uint axis = level % dim;
	    if((*node)->point.at(axis) < box[axis+dim] && (*node)->point.at(axis) > box[axis])
        {
	        float distance = sqrt(pow(target[0] - (*node)->point.at(0), 2) + pow(target[1] - (*node)->point.at(1), 2));
	        if(distance <= distanceTol)
                ids.push_back((*node)->id);
        }
        if((*node)->point.at(axis) > box[axis] && (*node)->left != NULL)
            searchHelper(box, &((*node)->left), ids, level+1, target, distanceTol);
        if((*node)->point.at(axis) < box[axis+dim] && (*node)->right != NULL) {
            searchHelper(box, &((*node)->right), ids, level+1, target, distanceTol);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::vector<float> box;
		for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) - distanceTol);
        }
        for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) + distanceTol);
        }
		searchHelper(box, &root, ids, 0, target, distanceTol);
		return ids;
	}

    std::vector<int> search(pcl::PointXYZ point, float distanceTol)
    {
        std::vector<int> ids;
        std::vector<float> target;
        target.push_back(point.x);
        target.push_back(point.y);
        target.push_back(point.z);
        std::vector<float> box;
        for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) - distanceTol);
        }
        for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) + distanceTol);
        }
        searchHelper(box, &root, ids, 0, target, distanceTol);
        return ids;
    }

    std::vector<int> search(pcl::PointXYZI point, float distanceTol)
    {
        std::vector<int> ids;
        std::vector<float> target;
        target.push_back(point.x);
        target.push_back(point.y);
        target.push_back(point.z);
        std::vector<float> box;
        for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) - distanceTol);
        }
        for(int i {0}; i < dim; i++)
        {
            box.push_back(target.at(i) + distanceTol);
        }
        searchHelper(box, &root, ids, 0, target, distanceTol);
        return ids;
    }

};




