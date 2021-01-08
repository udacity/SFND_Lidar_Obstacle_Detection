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

	KdTree()
		: root(NULL)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root,0, point, id );
	}

	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id ){
		//base case
		if(node == NULL){
			node = new Node(point, id);
		}else{

			uint dimension = depth%point.size(); // (dimensions are target.size())
			if (point[dimension]< node-> point[dimension]){
				insertHelper(node->left, depth + 1, point, id);
			}else{
				insertHelper(node->right, depth + 1, point, id);
			}

		}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, 0, distanceTol, ids);
		return ids;
	}

	bool checkConditions(Node* node, std::vector<float> target, float distanceTol ){
		bool condition = true;
		for (int x=0; x< target.size(); x++){
			condition = condition && (node->point[x] >= (target[x] - distanceTol));
		}

		// if it's false, return false, else check the next part
		if (!condition){
			return condition;
		}

		for (int x=0; x< target.size(); x++){
			condition = condition && (node->point[x] <= (target[x] + distanceTol));
		}

		return condition;
	}

	void searchHelper(Node* node, std::vector<float> target, uint depth, float distanceTol, std::vector<int> &ids ){
		// base case
		if(node != NULL){
			if (checkConditions(node, target, distanceTol)){
	
				float distance = 0.0;
				for (int x=0; x< target.size(); x++){
					distance += pow(node->point[x] - target[x], 2);
				}
				distance = sqrtf(distance);
				if (distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			if ((target[depth%target.size()] - distanceTol) < node->point[depth%target.size()]){
				searchHelper(node->left, target, depth+1, distanceTol, ids);
			}

			if ((target[depth%target.size()] + distanceTol) > node->point[depth%target.size()]){
				searchHelper(node->right, target, depth+1, distanceTol, ids);
			}

		}
	}
};
