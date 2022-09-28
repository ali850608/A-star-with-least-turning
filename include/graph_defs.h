#pragma once

#include <boost/config.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <string>

struct DotVertex
{
  bool isTwoWay;
  double position_x;
  double position_y;
  double outgoing_weight;

  std::string label;
  std::string source;
  std::string target;
};

struct DotEdge
{
  double weight = 1;
  double width = 0;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, DotVertex, DotEdge>
  NavGraph;
typedef boost::graph_traits<NavGraph>::adjacency_iterator AdjacencyIterator;

typedef boost::graph_traits<NavGraph>::vertex_descriptor VertexDesc;
typedef boost::graph_traits<NavGraph>::edge_descriptor EdgeDesc;
typedef boost::graph_traits<NavGraph>::vertex_iterator VertexIter;
typedef boost::graph_traits<NavGraph>::edge_iterator EdgeIter;

typedef boost::property_map<NavGraph, boost::vertex_index_t>::type IndexMap;
typedef boost::property_map<NavGraph, double DotEdge::*>::type WeightMap;
typedef boost::property_map<NavGraph, double DotEdge::*>::type WidthMap;
typedef boost::property_map<NavGraph, std::string DotVertex::*>::type NameMap;
typedef boost::property_map<NavGraph, double DotVertex::*>::type CoordMap;

typedef std::vector<VertexDesc> GraphPathType;
