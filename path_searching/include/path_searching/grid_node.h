#ifndef _GRID_NODE_H_
#define _GRID_NODE_H_

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

#define inf 1 >> 30

enum NODE_STATE { IN_CLOSE_SET = 1, IN_OPEN_SET = 2, NOT_EXPAND = 3 };

class GridNode;
typedef GridNode* GridNodePtr;

class GridNode {
 protected:
  int             rounds_{0};  // Distinguish every call
  double          g_score_{inf}, f_score_{inf};
  GridNodePtr     parent_{nullptr};
  enum NODE_STATE node_state_ { NOT_EXPAND };
  Eigen::Vector3i index_;

 public:
  void setNodeState(const enum NODE_STATE &state) { node_state_ = state; }
  void setIndex(const Eigen::Vector3i &idx) { index_ = idx; }
  void setRounds(const int &r) { rounds_ = r; }
  void setGScore(const double &g) { g_score_ = g; }
  void setFScore(const double &f) { f_score_ = f; }
  void setState(const enum NODE_STATE &s) { node_state_ = s; }
  void setParent(GridNode *p) { parent_ = p; }

  inline const double &getFScore() { return this->f_score_; }
  inline const double &getGScore() { return this->g_score_; }
  inline const int &   getRounds() { return this->rounds_; }

  inline const enum NODE_STATE &getNodeState() { return this->node_state_; }

  inline const GridNodePtr getParent() const { return this->parent_; }

  inline int             getIndex(int i) const { return index_(i); }
  inline Eigen::Vector3i getIndex() const { return index_; }
};

class NodeComparator {
 public:
  bool operator()(GridNodePtr node1, GridNodePtr node2) {
    return node1->getFScore() > node2->getFScore();
  }
};

#endif  // _GRID_NODE_H_