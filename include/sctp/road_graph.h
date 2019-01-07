#ifndef SCTP_PARSER_H_
#define SCTP_PARSER_H_
#include <string>
#include <vector>
#define MAX_NUM_INTER 300
#define MAX_NUM_ROADS (MAX_NUM_INTER * (MAX_NUM_INTER) / 2)
namespace sctp {

typedef unsigned long long int Minutes;

// Simple structure holding information about a road a specific intersection.
struct Road {
  Minutes time;       // time to traverse the road
  float probability;  // probability of traversing this road
  int next_inter;     // next intersection that you'll end up at if you traverse
                      // this road
};

class AliasTable {
 public:
  AliasTable(const std::vector<Road> &roads);
  Road Sample();

 private:
  std::vector<Road> roads_;
  std::vector<int> alias_;
  std::vector<float> heights_;
};

// Simple structrue holding the mean minutes that take each courier to deliver
// the package. If the value < 0 then the courier can't reach the specified
// destination.
struct CouriersMeanSec {
  float mean_sec_a;
  float mean_sec_b;
};

class RoadGraph {
 public:
  RoadGraph();
  bool Parse(const std::string &path);
  Road SampleNextRoad(int current_intersection);
  float SimulateTraversal(int start_inter, int end_inter, int iterations = 1);
  CouriersMeanSec SimulateTraversalCouriers(int iterations);
  // Check if the two intersections are connected somehow.
  // \param start the starting intersection
  // \param end the ending intersection
  // \return true if there is a path between the intersections
  bool IsReachable(int start, int end);
  void IsReachableAux(int start, int end, std::vector<bool> &visited);

 private:
  int num_inter_;
  int num_roads_;
  int starting_inter_a_;
  int starting_inter_b_;
  int ending_inter_;
  std::vector<std::vector<Road>> inter_;
  std::vector<AliasTable> alias_tables_;
};
}  // namespace sctp
#endif
