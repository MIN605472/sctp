#include "sctp/road_graph.h"
#include <fstream>
#include <future>
#include <iostream>
#include <random>
#include <thread>

using namespace sctp;

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
  if (!file.is_open()) {
    return false;
  }
  // Read the header info
  file >> num_inter_ >> num_roads_ >> ending_inter_ >> starting_inter_a_ >>
      starting_inter_b_;
  if (file.fail()) {
    file.close();
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
      file.close();
      return false;
    }
    Road road_u;
    road_u.next_inter = v;
    road_u.time = t_uv;
    road_u.probability = p_uv;
    Road road_v;
    road_v.next_inter = u;
    road_v.time = t_uv;
    road_v.probability = p_vu;
    inter_[u].push_back(road_u);
    inter_[v].push_back(road_v);
  }
  file.close();
  return true;
}

float RandomNumber() {
  static std::mt19937 mt((std::random_device())());
  static std::uniform_real_distribution<float> dist(0.0, 1.0);
  return dist(mt);
}

Road RoadGraph::SampleNextRoad(int current_intersection) {
  float epsilon = RandomNumber();
  float acc = 0;
  const std::vector<Road> &roads = inter_[current_intersection];
  // std::cerr << std::this_thread::get_id() << " Inter.: " <<
  // current_intersection
  //           << '\n';
  // std::cerr << std::this_thread::get_id() << " Epsilon: " << epsilon << '\n';
  // std::cerr << std::this_thread::get_id() << " Roads size: " << roads.size()
  //           << '\n';
  for (int i = 0; i < roads.size() - 1; ++i) {
    acc += roads[i].probability;
    // std::cerr << std::this_thread::get_id() << " Acc.: " << acc << '\n';
    if (epsilon <= acc) {
      return roads[i];
    }
  }
  // Sometimes the probabilites don't add up to exactly 1, but very close to it.
  // So we simply give the remaining probability to the last element. For
  // example, the total probability can be 0.9937 and our epsilon could be
  // 0.9942; thus the sampled element is the last one.
  return roads.back();
}

float RoadGraph::SimulateTraversal(int start_inter, int end_inter,
                                   int iterations) {
  if (!IsReachable(start_inter, end_inter)) {
    return -1;
  }
  Minutes total_minutes = 0;
  for (int i = 0; i < iterations; ++i) {
    for (int current_inter = start_inter; current_inter != end_inter;) {
      Road road = SampleNextRoad(current_inter);
      total_minutes += road.time;
      current_inter = road.next_inter;
    }
  }
  return (float)total_minutes / iterations;
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
