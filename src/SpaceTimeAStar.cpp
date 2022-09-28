#include "SpaceTimeAStar.h"

void SpaceTimeAStar::updatePath(const LLNode * goal, std::vector<PathEntry> & path)
{
  const LLNode * curr = goal;
  if (curr->is_goal) curr = curr->parent;
  path.reserve(curr->g_val + 1);
  while (curr != nullptr) {
    path.emplace_back(curr->location);
    curr = curr->parent;
  }
  std::reverse(path.begin(), path.end());
}

Path SpaceTimeAStar::findOptimalPath(
  const NavGraph & graph, VertexDesc start_location, VertexDesc goal_location)
{
  Path path;
  auto my_heuristic = compute_heuristics(graph, goal_location);

  // generate start and add it to the OPEN & FOCAL list
  auto start = new AStarNode(start_location, 0, 0, nullptr, 0);

  start->open_handle = open_list.push(start);
  start->in_openlist = true;
  allNodes_table.insert(start);

  while (!open_list.empty()) {
    auto * curr = popNode();
    assert(curr->location >= 0);
    // check if the popped node is a goal
    if (curr->location == goal_location) {
      updatePath(curr, path);
      break;
    }
    for (VertexDesc next_location : get_adjacent_locations(graph, curr->location)) {
      int next_timestep = curr->timestep + 1;
      int next_g_val = curr->g_val + getTwoVertexWeight(graph, curr->location, next_location);
      if (
        curr->parent != nullptr &&
        isTurning(graph, curr->parent->location, curr->location, next_location))
        next_g_val += 500;  // reduce turning weight
      int next_h_val = my_heuristic[next_location];

      // generate (maybe temporary) node
      auto next = new AStarNode(next_location, next_g_val, next_h_val, curr, next_timestep);

      // try to retrieve it from the hash table
      auto it = allNodes_table.find(next);
      if (it == allNodes_table.end()) {
        pushNode(next);
        allNodes_table.insert(next);
        continue;
      }
      // update existing node's if needed (only in the open_list)

      auto existing_next = *it;
      if (existing_next->getFVal() > next->getFVal()) {
        existing_next->copy(*next);       // update existing node
        if (!existing_next->in_openlist)  // if its in the closed list (reopen)
        {
          pushNode(existing_next);
        } else {
          open_list.increase(existing_next->open_handle);  // increase because #conflicts improved
        }
      }
      delete (next);  // not needed anymore -- we already generated it before
    }                 // end for loop that generates successors
  }                   // end while loop

  releaseNodes();
  return path;
}

inline AStarNode * SpaceTimeAStar::popNode()
{
  AStarNode * node;
  node = open_list.top();
  open_list.pop();

  node->in_openlist = false;
  return node;
}

inline void SpaceTimeAStar::pushNode(AStarNode * node)
{
  node->open_handle = open_list.push(node);
  node->in_openlist = true;
}

void SpaceTimeAStar::releaseNodes()
{
  open_list.clear();
  for (auto node : allNodes_table) delete node;
  allNodes_table.clear();
}

std::vector<int> SpaceTimeAStar::compute_heuristics(
  const NavGraph & graph, VertexDesc goal_location)
{
  struct Node
  {
    int location;
    int value;

    Node() = default;
    Node(VertexDesc location, int value) : location(location), value(value) {}
    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
      // returns true if n1 > n2 (note -- this gives us *min*-heap).
      bool operator()(const Node & n1, const Node & n2) const { return n1.value >= n2.value; }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)
  };
  std::vector<int> my_heuristic;
  my_heuristic.resize(boost::num_vertices(graph), MAX_TIMESTEP);

  // generate a heap that can save nodes (and a open_handle)
  boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node> > heap;

  Node root(goal_location, 0);
  my_heuristic[goal_location] = 0;
  heap.push(root);  // add root to heap
  while (!heap.empty()) {
    Node curr = heap.top();
    heap.pop();
    for (VertexDesc next_location : get_adjacent_locations(graph, curr.location)) {
      if (
        my_heuristic[next_location] >
        curr.value + getTwoVertexWeight(graph, curr.location, next_location)) {
        my_heuristic[next_location] =
          curr.value + getTwoVertexWeight(graph, curr.location, next_location);
        Node next(
          next_location, curr.value + getTwoVertexWeight(graph, curr.location, next_location));
        heap.push(next);
      }
    }
  }

  return my_heuristic;
}