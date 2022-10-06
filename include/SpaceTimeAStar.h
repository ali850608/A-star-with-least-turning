#pragma once
#include "graph_utils.h"

class LLNode  // low-level node
{
public:
  VertexDesc location;
  int g_val;
  int h_val = 0;
  LLNode * parent;
  bool in_openlist = false;
  bool wait_at_goal;
  // the action is to wait at the goal vertex or not. This is used for >lenghth constraints
  bool is_goal = false;
  // the following is used to comapre nodes in the OPEN list
  struct compare_node
  {
    // returns true if n1 > n2 (note -- this gives us *min*-heap).
    bool operator()(const LLNode * n1, const LLNode * n2) const
    {
      if (n1->g_val + n1->h_val == n2->g_val + n2->h_val) {
        if (n1->h_val == n2->h_val) {
          return rand() % 2 == 0;  // break ties randomly
        }
        return n1->h_val >= n2->h_val;
        // break ties towards smaller h_vals (closer to goal location)
      }
      return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
    }
  };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

  LLNode()
  : location(0), g_val(0), h_val(0), parent(nullptr), in_openlist(false), wait_at_goal(false)
  {
  }

  LLNode(int location, int g_val, int h_val, LLNode * parent, bool in_openlist = false)
  : location(location),
    g_val(g_val),
    h_val(h_val),
    parent(parent),
    in_openlist(in_openlist),
    wait_at_goal(false)
  {
  }

  inline int getFVal() const { return g_val + h_val; }
  void copy(const LLNode & other)
  {
    location = other.location;
    g_val = other.g_val;
    h_val = other.h_val;
    parent = other.parent;
    wait_at_goal = other.wait_at_goal;
    is_goal = other.is_goal;
  }
};

class AStarNode : public LLNode
{
public:
  // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
  typedef boost::heap::pairing_heap<
    AStarNode *, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle_t;

  open_handle_t open_handle;

  AStarNode() : LLNode() {}

  AStarNode(VertexDesc loc, int g_val, int h_val, LLNode * parent, bool in_openlist = false)
  : LLNode(loc, g_val, h_val, parent, in_openlist)
  {
  }

  ~AStarNode() {}

  // The following is used by for generating the hash value of a nodes
  struct NodeHasher
  {
    size_t operator()(const AStarNode * n) const
    {
      size_t loc_hash = std::hash<int>()(n->location);
      return (loc_hash ^ 2);
    }
  };

  // The following is used for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and agree on the id and timestep
  struct eqnode
  {
    bool operator()(const AStarNode * s1, const AStarNode * s2) const
    {
      return (s1 == s2) ||
             (s1 && s2 && s1->location == s2->location && s1->wait_at_goal == s2->wait_at_goal);
    }
  };
};

class SpaceTimeAStar
{
protected:
  int min_f_val;  // minimal f value in OPEN

  std::vector<int> compute_heuristics(const NavGraph & graph, VertexDesc goal_location);

public:
  // find path by time-space A* search
  // Returns a shortest path that satisfies the constraints of the give node  while
  // minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
  // lowerbound is an underestimation of the length of the path in order to speed up the search.
  Path findOptimalPath(const NavGraph & graph, VertexDesc start_location, VertexDesc goal_location);

  // SpaceTimeAStar() {}
  // ~SpaceTimeAStar() {}

private:
  // define typedefs and handles for heap
  typedef boost::heap::pairing_heap<AStarNode *, boost::heap::compare<AStarNode::compare_node> >
    heap_open_t;

  heap_open_t open_list;

  // define typedef for hash_map
  typedef boost::unordered_set<AStarNode *, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
  hashtable_t allNodes_table;

  bool optimal = true;

  // Updates the path datamember
  static void updatePath(const LLNode * goal, std::vector<PathEntry> & path);
  void updateFocalList();
  inline AStarNode * popNode();
  inline void pushNode(AStarNode * node);
  void releaseNodes();
};
