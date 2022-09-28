#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <tuple>

#include "SpaceTimeAStar.h"

int main(int argc, char * argv[])
{
  NavGraph graph_;
  std::filesystem::path mapFile = std::filesystem::current_path() / "map/twenty_robots.dot";
  std::string graph_name_ = loadGraph(graph_, mapFile.string());

  std::optional<VertexDesc> v1 = getVertexFromLabel(graph_, "l06");
  std::optional<VertexDesc> v2 = getVertexFromLabel(graph_, "r01");

  SpaceTimeAStar * a_star = new SpaceTimeAStar();

  auto path = a_star->findOptimalPath(graph_, *v1, *v2);
  std::cout << path.size() << std::endl;
  for (const auto & v : path) std::cout << getVertexLabel(graph_, v.location) << ";";

  delete a_star;

  return 0;
}