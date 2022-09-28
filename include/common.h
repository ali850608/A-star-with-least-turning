#pragma once
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <fstream>
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::fixed
#include <list>
#include <set>
#include <stack>
#include <tuple>
#include <vector>

#include "graph_defs.h"

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
  VertexDesc location = -1;
  explicit PathEntry(VertexDesc loc = -1) : location(loc) {}
};

typedef std::vector<PathEntry> Path;
std::ostream & operator<<(std::ostream & os, const Path & path);

bool isSamePath(const Path & p1, const Path & p2);
