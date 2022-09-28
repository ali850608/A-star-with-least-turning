#pragma once
#include <cmath>
#include <optional>
#include <sstream>
#include <utility>

#include "common.h"

std::string loadGraph(NavGraph & graph, std::string dotfile);

std::optional<VertexDesc> getVertexFromLabel(const NavGraph & graph, const std::string & label);

std::list<VertexDesc> get_adjacent_locations(const NavGraph & graph, VertexDesc vertex);

std::string getVertexLabel(const NavGraph & graph, VertexDesc vertex);

int getTwoVertexWeight(const NavGraph & graph, VertexDesc from, VertexDesc to);

bool isTurning(const NavGraph & graph, VertexDesc parent, VertexDesc curr, VertexDesc child);