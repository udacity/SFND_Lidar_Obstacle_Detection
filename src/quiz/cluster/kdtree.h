/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <queue>
#include <utility>

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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert_helper(root, point, id, 0);
	}

	void insert_helper(Node *&current, std::vector<float> point, int id, int level){
		if(!current){
			current = new Node(point, id);
			return;
		}
		else{
			if(level % 2 == 0){
				if(point[0] <= current->point[0]){
					insert_helper(current->left, point, id, ++level);
				}
				else{
					insert_helper(current->right, point, id, ++level);
				}
			}
			else{
				if(point[1] <= current->point[1]){
					insert_helper(current->left, point, id, ++level);
				}
				else{
					insert_helper(current->right, point, id, ++level);
				}
			}
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		float x_min = target[0]-distanceTol;
		float x_max = target[0]+distanceTol;
		float y_min = target[1]-distanceTol;
		float y_max = target[1]+distanceTol;
		std::queue<std::pair<Node*, int> > to_go_queue;


		// Return 1 to check only left subtree
		// Return 0 to check both subtree
		// Return -1 to check right subtree
		auto check_range = [x_min, x_max, y_min, y_max](std::vector<float> point, int l) -> int{
			if(l%2 == 0){
				if(x_min <= point[0] && x_max >= point[0]){
					return 0;
				}
				else if(point[0] >= x_min ){
					return -1;
				}
				else{
					return 1;
				}
				
			}
			else{
				if(y_min <= point[1] && y_max >= point[1]){
					return 0;
				}
				else if(point[1] >= y_min ){
					return -1;
				}
				else{
					return 1;
				}
			}
		};

		auto is_proximal = [=](std::vector<float> point) -> bool{

			if(point[0] >= x_min && point[0] <= x_max && point[1] >= y_min && point[1] <= y_max){
				if(sqrt( pow((point[0] - target[0]), 2) + pow((point[1] -target[1]),2) ) <= distanceTol){
					return true;
				}
			}
			return false;

		};

		to_go_queue.push(std::make_pair(root, 0));
		
		while(!to_go_queue.empty()){
			auto entry = to_go_queue.front();
			to_go_queue.pop();
			int level = entry.second;
			Node* current =  entry.first;

			if(!current){
				continue;
			}

			if(is_proximal(current->point)){
				ids.push_back(current->id);
			}

			auto range = check_range(current->point, level);

			if(range == 1){
				to_go_queue.push(std::make_pair(current->left, ++level));
			}
			else if(range == 0){
				to_go_queue.push(std::make_pair(current->left, ++level));
				to_go_queue.push(std::make_pair(current->right, ++level));
			}
			else{
				to_go_queue.push(std::make_pair(current->right, ++level));
			}
		}

		return ids;
	}
	

};




