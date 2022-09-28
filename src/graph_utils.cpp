#include "graph_utils.h"

std::string loadGraph(NavGraph & graph, std::string dotfile)
{
  boost::dynamic_properties dp(boost::ignore_other_properties);

  dp.property("node_id", boost::get(&DotVertex::label, graph));
  dp.property("label", boost::get(&DotVertex::label, graph));
  dp.property("position_x", boost::get(&DotVertex::position_x, graph));
  dp.property("position_y", boost::get(&DotVertex::position_y, graph));

  dp.property("weight", boost::get(&DotEdge::weight, graph));
  dp.property("width", boost::get(&DotEdge::width, graph));

  std::ifstream dot(dotfile);
  boost::read_graphviz(dot, graph, dp);

  // Initialize coordinates tree and mutual exvlusion
  CoordMap position_x = boost::get(&DotVertex::position_x, graph);
  CoordMap position_y = boost::get(&DotVertex::position_y, graph);
  IndexMap index = boost::get(boost::vertex_index, graph);
  WeightMap weight_map = boost::get(&DotEdge::weight, graph);

  boost::graph_traits<NavGraph>::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
    const double x_diff =
      position_x[index[boost::source(*ei, graph)]] - position_x[index[boost::target(*ei, graph)]];
    const double y_diff =
      position_y[index[boost::source(*ei, graph)]] - position_y[index[boost::target(*ei, graph)]];
    double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff) * 100 * weight_map[*ei];
    boost::put(boost::get(&DotEdge::weight, graph), *ei, (int)distance);
  }
  // Return graph name
  if (dotfile.substr(dotfile.size() - 4) == ".dot") {
    std::string graph_file_name = dotfile.substr(dotfile.find_last_of("/\\") + 1);
    std::string::size_type const p(graph_file_name.find_last_of('.'));
    std::string graph_name = graph_file_name.substr(0, p);
    return graph_name;
  }
  return "";
}

std::optional<VertexDesc> getVertexFromLabel(const NavGraph & graph, const std::string & label)
{
  std::optional<VertexDesc> rtn;

  auto label_index = boost::get(&DotVertex::label, graph);
  boost::graph_traits<NavGraph>::vertex_iterator vi, vi_end;
  for (boost::tie(vi, vi_end) = boost::vertices(graph); vi != vi_end; ++vi) {
    if (label_index[*vi] == label) {
      rtn = vertex(*vi, graph);
      return rtn;
    }
  }
  return rtn;
}

std::list<VertexDesc> get_adjacent_locations(const NavGraph & graph, VertexDesc vertex)
{
  std::list<VertexDesc> locations;
  AdjacencyIterator ai, a_end;
  boost::tie(ai, a_end) = boost::adjacent_vertices(vertex, graph);
  for (; ai != a_end; ai++) {
    locations.emplace_back(*ai);
  }
  return locations;
}

std::string getVertexLabel(const NavGraph & graph, VertexDesc vertex)
{
  return boost::get(boost::get(&DotVertex::label, graph), vertex);
}

int getTwoVertexWeight(const NavGraph & graph, VertexDesc from, VertexDesc to)
{
  auto path_edge = boost::edge(from, to, graph);
  if (path_edge.second) return boost::get(boost::get(&DotEdge::weight, graph), path_edge.first);
  return 0;
}

bool isTurning(const NavGraph & graph, VertexDesc parent, VertexDesc curr, VertexDesc child)
{
  auto x1 = boost::get(boost::get(&DotVertex::position_x, graph), parent);
  auto y1 = boost::get(boost::get(&DotVertex::position_y, graph), parent);
  auto x2 = boost::get(boost::get(&DotVertex::position_x, graph), curr);
  auto y2 = boost::get(boost::get(&DotVertex::position_y, graph), curr);
  auto x3 = boost::get(boost::get(&DotVertex::position_x, graph), child);
  auto y3 = boost::get(boost::get(&DotVertex::position_y, graph), child);

  // vector AB
  std::pair<double, double> AB;
  AB.first = x2 - x1;
  AB.second = y2 - y1;

  std::pair<double, double> BC;
  BC.first = x3 - x2;
  BC.second = y3 - y2;

  double dot = AB.first * BC.first + AB.second * BC.second;
  double det = AB.first * BC.second - AB.second * BC.first;
  double angle = std::abs(std::atan2(det, dot));

  if (angle < 2.96 && angle > 0.17) return true;
  return false;
}
