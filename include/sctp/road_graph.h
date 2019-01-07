#ifndef SCTP_PARSER_H_
#define SCTP_PARSER_H_
#include <string>
#include <vector>
#define MAX_NUM_INTER 300
#define MAX_NUM_ROADS (MAX_NUM_INTER * (MAX_NUM_INTER) / 2)
namespace sctp {

typedef unsigned long long int Minutes;

// Simple structure holding information about a road of a specific intersection.
struct Road {
  Minutes time;       // time to traverse the road
  float probability;  // probability of traversing this road
  int next_inter;     // next intersection that you'll end up at if you traverse
                      // this road
};

// Class used for fast sampling.
class AliasTable {
 public:
  // Init the alias table.
  // \param roads the outgoing roads of an intersection
  AliasTable(const std::vector<Road> &roads);
  // Sample an outgoing road.
  // \return the sampled road
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

// Class representing the graph of roads and intersections.
class RoadGraph {
 public:
  RoadGraph();

  // Parse the graph description from a file.
  // \param path the path of the file
  // \return true if the file was parsed correctly, false otherwise
  bool Parse(const std::string &path);

  // Sample the next road to take from a given intersection.
  // \param current_intersection the intersection from which we want to sample a
  // road \return the sampled road from the intersection \p current_intersection
  Road SampleNextRoad(int current_intersection);

  // Simulate a traversal from a starting to and ending intersection.
  // \param start_inter the starting intersection
  // \param end_inter the ending intersection
  // \param iterations the number of times to repeat the traversal
  // \return the mean time that takes to go from \p start_inter to \p end_inter
  float SimulateTraversal(int start_inter, int end_inter, int iterations = 1);

  // Simulate the traversal of the two couriers.
  // \param iterations the number of times to repeat the traversal
  // \return the mean time of each courier
  CouriersMeanSec SimulateTraversalCouriers(int iterations);

  // Check if the two intersections are connected somehow.
  // \param start the starting intersection
  // \param end the ending intersection
  // \return true if there is a path between the intersections
  bool IsReachable(int start, int end);

 private:
  void IsReachableAux(int start, int end, std::vector<bool> &visited);

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
