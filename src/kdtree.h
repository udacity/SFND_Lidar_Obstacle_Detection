/* \author Aaron Brown */
// Quiz on implementing kd tree


#ifndef PLAYBACK_KDTREE_H
#define PLAYBACK_KDTREE_H

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
//#include <chrono>
#include <math.h>

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree {
    Node* root;
    unsigned int kDim=2;//3 //todo read dynamically 3d or 2d -- from argument = std::vector<float> point
    KdTree() : root(NULL) {}

    void _inserthelper(Node ** node, unsigned int depth, std::vector<float> point, int id ) {
        if (*node == NULL) {
            kDim = point.size();
//            std::cout << "insert dim kDim='" << kDim << "'" << std::endl;
            *node = new Node(point, id);
        } else {
            unsigned int cd = depth % kDim; //kDim=2 x (even = 0 index) or y (odd = 1 index). for kDim=3 cd=0,1,2
            if(point[cd] < (*node)->point[cd])
                _inserthelper(&((*node)->left), depth+1, point, id );
            else
                _inserthelper(&((*node)->right), depth+1, point, id );
        }
    }


    void insert(std::vector<float> point, int id) {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        _inserthelper(&root, 0, point, id);

    }

    bool _checkPointInTol(std::vector<float> target, Node* node, float distanceTol, std::vector<int>& ids) {
        float distance =0;
        for(int iDim=0; iDim<kDim; iDim++){
            //if( node->point[iDim] >= (target[iDim] - distanceTol) && node->point[iDim] <= (target[iDim] + distanceTol)){
            float distDim = fabs(target[iDim] -  node->point[iDim]);
            if(distDim <= distanceTol){
                distance += distDim * distDim;
                //continue if within range
                continue;
            }
            //exit if out of range
            return false;
        }
        //check sqrt of  all Dim in range
        bool inRange = sqrt(distance) <= distanceTol ;
        if ( inRange)
            ids.push_back(node->id);
        return inRange;
    }

    void _searchHelper(std::vector<float> target, Node* node, unsigned int depth, float distanceTol, std::vector<int>& ids) {
        if (node != NULL) return;

        //is within range distanceTol?
        _checkPointInTol(target,node, distanceTol,ids);

        //continue with tree search on either side
        unsigned int cd = depth % kDim;
        //check other points : minus distanceTol
        if ((target[cd] - distanceTol) < node->point[cd])
            _searchHelper(target, node->left, depth + 1, distanceTol, ids);
        //check other points : plus distanceTol
        if ((target[cd] + distanceTol) > node->point[cd])
            _searchHelper(target, node->right, depth + 1, distanceTol, ids);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> ids;
//        kDim = target.size() ;
//        std::cout << "search dim kDim='" << kDim << "'" << std::endl;
        _searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }


};





#endif //PLAYBACK_KDTREE_H