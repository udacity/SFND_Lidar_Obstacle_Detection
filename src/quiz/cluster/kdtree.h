/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @create date 2021-02-02 09:55:24
 * @modify date 2021-02-02 09:55:24
 * @desc KD-Tree implementation
 */
/* quiz/starter code author Aaron Brown */

#ifndef KDTREE_H
#define KDTREE_H

#include "../../render/render.h"
#include <exception>
#include <numeric>

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  /**
   * Insert recursively a point in the tree node
   * @param node: The tree (or subtree/node) where the point should be inserted
   * @param depth: Current depth of the subtree/node (0 is the root)
   * @param point: The point to be inserted.
   * @param id: The id of the point.
   */
  void insert_rec(Node **node, uint depth, std::vector<float> point, int id) {
    // insert recursively the point in the node
    if (*node == NULL) {
      *node = new Node(point, id);
    } else {
      if ((*node)->point.size() == point.size()) {
        // the first point inserted in the tree defines the dimension of the
        // data then a point is only inserted when it is compatible with this
        // tree dimension !
        auto dim = depth % point.size();
        if (point[dim] < (*node)->point[dim]) {
          // insert to the left branch
          insert_rec(&((*node)->left), depth + 1, point, id);
        } else {
          // insert to the right branch
          insert_rec(&((*node)->right), depth + 1, point, id);
        }
      }
    }
  }

  void insert(std::vector<float> point, int id) {
    insert_rec(&root, 0, point, id);
  }

  /**
   * A helper to compute a squared euclidean distance between 2 vectors
   */

  float squared_euclidean_distance(std::vector<float> point,
                                   std::vector<float> target) {
    if (point.size() != target.size()) {
      throw "size mismatch !";
    }
    float acc = 0;
    for (unsigned int index = 0; index < point.size(); ++index)
      acc += pow((point[index] - target[index]), 2);
    return acc;
  }

  /** A helper to check if a point is in a target hyper-cube defined by
   distanceTol
   * if true -> compute the distance and decide about adding it to the range
   `distanceTol` result (ids)

   * @param node: tree node to decide about its proximity to the target (eq
   (*node)-> point is enough close to the target)
   * @param target: target point
   * @param distanceTol: distance tolerance
  */
  bool checkInHyperCube(std::vector<float> point, std::vector<float> target,
                        float distanceTol) {
    if (point.size() != target.size()) {
      throw "size mismatch !";
    }
    bool in = false;
    int index = 0;
    while (index < target.size()) {
      if (((target[index] - distanceTol) > point[index]) ||
          ((target[index] + distanceTol) < point[index])) {
        return false;
      }
      index++;
    }
    return index == target.size(); // also return True should work
  }

  void checkAndAdd(Node *node, std::vector<float> target, float distanceTol,
                   std::vector<int> &ids) {
    auto point = (node)->point;
    // in the hyper-cube (box) -> compute distance -> include or not in the
    // result (ids)
    if (checkInHyperCube(point, target, distanceTol)) {
      if (squared_euclidean_distance(point, target) <
          (distanceTol * distanceTol)) {
        ids.push_back((node)->id);
      }
    }
  }

  void search_rec(Node *node, uint depth, std::vector<int> &ids,
                  std::vector<float> target, float distanceTol) {
    if (node != NULL) {
      checkAndAdd(node, target, distanceTol, ids);
      auto dim = depth % target.size();
      // an exception would be raised by checkAndAdd
      // if the dimension mismatch (so it is safe to
      // use the target.size())
      auto point = (node)->point;
      if ((target[dim] - distanceTol) < point[dim]) {
        // probable close enough nodes in the right side of the current one
        search_rec((node)->left, depth+1, ids, target, distanceTol);
      }
      if ((target[dim] + distanceTol) > point[dim]) {
        // probable close enough nodes in the right side of the current one
        search_rec((node)->right, depth+1, ids, target, distanceTol);
      }
    }
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    search_rec(root, 0, ids, target, distanceTol);
    return ids;
  }
};

#endif
