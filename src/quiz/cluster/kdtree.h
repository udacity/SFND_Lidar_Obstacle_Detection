/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree() : root(NULL)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if (root == NULL)
		{
			root = new Node(point, id);
			return;
		}

		// traverse and determine where to insert. And keep track of depth
		unsigned depth = 0;
		Node *currentNode = root;

		while (true)
		{
			// compare x
			if (depth % 2 == 0)
			{
				if (point[0] < currentNode->point[0])
				{
					if (currentNode->left == NULL)
					{
						currentNode->left = new Node(point, id);
						break;
					}
					currentNode = currentNode->left;
				}
				else
				{
					if (currentNode->right == NULL)
					{
						currentNode->right = new Node(point, id);
						break;
					}
					currentNode = currentNode->right;
				}
			}
			// compare y
			else
			{
				if (point[1] < currentNode->point[1])
				{
					if (currentNode->left == NULL)
					{
						currentNode->left = new Node(point, id);
						break;
					}
					currentNode = currentNode->left;
				}
				else
				{
					if (currentNode->right == NULL)
					{
						currentNode->right = new Node(point, id);
						break;
					}
					currentNode = currentNode->right;
				}
			}
			depth++;
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
