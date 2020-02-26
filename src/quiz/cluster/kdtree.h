/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>

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

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id) {
		insert_recursive(root, point, id, 0);
	}

	void insert_recursive(Node *&node, std::vector<float> point, int id, int depth)	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (node == NULL) {
			node = new Node(point, id);
		}
		else {
			// even ids based on X-axis
			int cd = depth % 2;
			if (point[cd] > node->point[cd]) {
				insert_recursive(node->right, point, id, depth + 1);
			}
			else {
				insert_recursive(node->left, point, id, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(ids, root, target, distanceTol, 0);
		return ids;
	}

	void search_helper(std::vector<int> &ids, Node* node, std::vector<float> target, float distanceTol, int depth)
	{
		if (node != NULL) {
			int cd = depth % 2;
			float distance = sqrt(pow((target[0] - node->point[0]), 2) + pow((target[1] - node->point[1]), 2));
			if (distance < distanceTol) {
				ids.push_back(node->id);
			}
			if (node->point[cd] >= target[cd] - distanceTol) {
				search_helper(ids, node->left, target, distanceTol, depth + 1);
			}
			if (node->point[cd] <= target[cd] + distanceTol) {
				search_helper(ids, node->right, target, distanceTol, depth + 1);
			}
		}
	}
};




