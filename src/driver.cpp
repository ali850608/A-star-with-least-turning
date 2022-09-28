#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <tuple>

#include "SpaceTimeAStar.h"

int main(int argc, char * argv[])
{
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
    "map,m", po::value<std::string>()->required(), "map name in map folder")(
    "start,s", po::value<std::string>()->required(), "start label")(
    "goal,g", po::value<std::string>()->required(), "goal label");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  std::string file_dir = "map/" + vm["map"].as<std::string>() + ".dot";
  NavGraph graph_;
  std::filesystem::path mapFile = std::filesystem::current_path() / file_dir;
  std::string graph_name_ = loadGraph(graph_, mapFile.string());

  std::optional<VertexDesc> v1 = getVertexFromLabel(graph_, vm["start"].as<std::string>());
  std::optional<VertexDesc> v2 = getVertexFromLabel(graph_, vm["goal"].as<std::string>());

  SpaceTimeAStar * a_star = new SpaceTimeAStar();

  auto path = a_star->findOptimalPath(graph_, *v1, *v2);
  std::cout << path.size() << std::endl;
  for (const auto & v : path) std::cout << getVertexLabel(graph_, v.location) << ";";

  delete a_star;

  return 0;
}