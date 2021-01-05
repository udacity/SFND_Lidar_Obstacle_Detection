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
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Create new node using constructor
		Node* node = new Node(point, id);

		// Use recursive insertHelper method to insert node in tree
		// Start from root and at depth 0 (default)
		_insertHelper(root, node);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		
		// Use recursive searchHelper method
		// Start from root and at depth 0 (default)
		// ids is an empty vector passed by reference and stores the final result
		_searchHelper(root, target, ids, distanceTol);
		return ids;
	}
	
	private:
		void _insertHelper(Node* &cur, Node* node, uint depth=0) 
		{
			// if current node pointer is nullptr, we have found the empty child position where to insert our node
			if (!cur) {
				cur = node;
				return;
			}

			// else compute current dimension: 0 if depth is even, 1 if depth is odd
			uint cd = depth % 2;
			// increment depth by 1
			depth++;
			// Left if smaller, right if greater
			if (node->point[cd] < cur->point[cd])
				_insertHelper(cur->left, node, depth);
			else
				_insertHelper(cur->right, node, depth);
		}

		void _searchHelper(Node* &cur, std::vector<float> &target, std::vector<int> &ids, float distanceTol, uint depth=0) 
		{
			// if current node pointer is nullptr, we can exit the function (no more comparisons are needed)
			if (cur) {
				// else compute current dimension: 0 if depth is even, 1 if depth is odd
				uint cd = depth % 2;
				// increment depth by 1
				depth++;
				// Check if current node is in the box (both differences in X and Y from target are lower in modulus than tolerance)
				float deltaX = cur->point[0] - target[0];
				float deltaY = cur->point[1] - target[1];
				if ( ((deltaX <= distanceTol) && (deltaX >= -distanceTol)) && ((deltaY <= distanceTol) && (deltaY >= -distanceTol)) ) 
				{
					// If so, check if distance is smaller than distanceTol and add current node to ids
					float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
					if (distance<=distanceTol)
						ids.push_back(cur->id);
				}
				// Search left if box lower boundary is smaller than the current node
				if (target[cd] - distanceTol <= cur->point[cd])
					_searchHelper(cur->left, target, ids, distanceTol, depth);
				// Search right if box upper boundary is greater than the current node
				if (target[cd] + distanceTol >= cur->point[cd])
					_searchHelper(cur->right, target, ids, distanceTol, depth);
			}
		}
};




