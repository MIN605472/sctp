#include "sctp/road_graph.h"
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <thread>

using namespace sctp;

template <typename T>
T RandomNumber(T min, T max) {
  static std::mt19937 mt((std::random_device())());
  std::uniform_real_distribution<T> dist(min, max);
  return dist(mt);
}

template <>
int RandomNumber<int>(int min, int max) {
  static std::mt19937 mt((std::random_device())());
  std::uniform_int_distribution<int> dist(min, max);
  return dist(mt);
}

AliasTable::AliasTable(const std::vector<Road> &roads)
    : roads_(roads), alias_(roads.size()), heights_(roads.size()) {
  float n = 1.0 / roads_.size();
  int g;
  int m;
  int mm;
  for (g = 0; g < roads_.size() && roads_[g].probability < n; ++g)
    ;
  for (m = 0; m < roads_.size() && roads_[m].probability >= n; ++m)
    ;
  mm = m + 1;
  while (g < roads_.size() && m < roads_.size()) {
    alias_[m] = g;
    heights_[m] = roads_[m].probability;
    roads_[g].probability -= n - roads_[m].probability;
    if (roads_[g].probability >= n || mm <= g) {
      for (m = mm; m < roads_.size() && roads_[m].probability >= n; ++m)
        ;
      mm = m + 1;
    } else {
      m = g;
    }
    for (; g < roads_.size() && roads_[g].probability < n; ++g)
      ;
  }
  for (; g < roads_.size(); ++g) {
    if (roads_[g].probability >= n) {
      heights_[g] = std::numeric_limits<float>::max();
      alias_[g] = g;
    }
  }
  if (m < roads_.size()) {
    heights_[m] = std::numeric_limits<float>::max();
    alias_[m] = m;
    for (m = mm; m < roads_.size(); ++m) {
      if (roads_[m].probability <= n) {
        heights_[m] = std::numeric_limits<float>::max();
        alias_[m] = m;
      }
    }
  }
}

Road AliasTable::Sample() {
  int u = RandomNumber<int>(0, roads_.size() - 1);
  float v = RandomNumber<float>(0.0, 1.0 / roads_.size());
  auto i = v < heights_[u] ? roads_[u] : roads_[alias_[u]];
  return i;
}

RoadGraph::RoadGraph()
    : num_inter_(-1),
      num_roads_(-1),
      starting_inter_a_(-1),
      starting_inter_b_(-1),
      ending_inter_(-1),
      inter_() {}

bool RoadGraph::Parse(const std::string &path) {
  std::ifstream file;
  file.open(path);
  if (file.fail()) {
    return false;
  }
  // Read the header info
  file >> num_inter_ >> num_roads_ >> ending_inter_ >> starting_inter_a_ >>
      starting_inter_b_;
  if (file.fail()) {
    return false;
  }
  inter_.resize(num_inter_);
  // Read the roads info
  for (int i = 0; i < num_roads_; ++i) {
    int u = -1;
    int v = -1;
    int t_uv = -1;
    float p_uv = -1;
    float p_vu = -1;
    file >> u >> v >> t_uv >> p_uv >> p_vu;
    if (file.fail()) {
      return false;
    }
    if (p_uv > 0) {
      Road road_u;
      road_u.next_inter = v;
      road_u.time = t_uv;
      road_u.probability = p_uv;
      inter_[u].push_back(road_u);
    }
    if (p_vu > 0) {
      Road road_v;
      road_v.next_inter = u;
      road_v.time = t_uv;
      road_v.probability = p_vu;
      inter_[v].push_back(road_v);
    }
  }
  alias_tables_.reserve(inter_.size());
  for (const auto &roads : inter_) {
    alias_tables_.emplace_back(roads);
  }
  return true;
}

Road RoadGraph::SampleNextRoad(int current_intersection) {
  return alias_tables_[current_intersection].Sample();
}

float RoadGraph::SimulateTraversal(int start_inter, int end_inter,
                                   int iterations) {
  if (!IsReachable(start_inter, end_inter)) {
    return -1;
  }
  Minutes total_minutes = 0;
  for (int i = 0; i < iterations; ++i) {
    for (int current_inter = start_inter; current_inter != end_inter;) {
      // std::cout << current_inter << ';';
      Road road = SampleNextRoad(current_inter);
      total_minutes += road.time;
      current_inter = road.next_inter;
    }
  }
  return total_minutes / (float)iterations;
}

CouriersMeanSec RoadGraph::SimulateTraversalCouriers(int iterations) {
  CouriersMeanSec means;
  auto courier_a = std::async(&RoadGraph::SimulateTraversal, this,
                              starting_inter_a_, ending_inter_, iterations);
  auto courier_b = std::async(&RoadGraph::SimulateTraversal, this,
                              starting_inter_b_, ending_inter_, iterations);
  means.mean_sec_a = courier_a.get();
  means.mean_sec_b = courier_b.get();
  return means;
}

bool RoadGraph::IsReachable(int start, int end) {
  std::vector<bool> visited(num_inter_, false);
  IsReachableAux(start, end, visited);
  return visited[end];
}

void RoadGraph::IsReachableAux(int start, int end, std::vector<bool> &visited) {
  visited[start] = true;
  for (const Road &r : inter_[start]) {
    if (!visited[r.next_inter]) {
      IsReachableAux(r.next_inter, end, visited);
    }
  }
}
