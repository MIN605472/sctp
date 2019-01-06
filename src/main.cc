#include <cstdlib>
#include <cstring>
#include <iostream>
#include "sctp/road_graph.h"

using namespace sctp;

static const char kUsage[] =
    R"(Usage:
  ./SCTP -i <int> -f <string>
Options:
  -i <int> the number of iterations
  -f <string> the input file
)";

int iterations;
std::string file_path;

// Parse the arguments to the main program.
// \param argc the number of args
// \param argv the arg list
// \return true if the args have been parsed correctly, false otherwise
bool ParseArgs(int argc, char *argv[]) {
  for (int i = 1; i < argc; i += 2) {
    if (strcmp(argv[i], "-i") == 0) {
      char *endptr = nullptr;
      iterations = strtol(argv[i + 1], &endptr, 10);
      if (iterations <= 0) {
        return false;
      }
    } else if (strcmp(argv[i], "-f") == 0) {
      file_path = argv[i + 1];
    } else {
      return false;
    }
  }
  return true;
}

int main(int argc, char *argv[]) {
  if (argc != 5 || !ParseArgs(argc, argv)) {
    std::cerr << kUsage;
    return 1;
  }
  RoadGraph parser;
  parser.Parse(file_path);
  CouriersMeanSec couriers_time = parser.SimulateTraversalCouriers(iterations);
  if (couriers_time.mean_sec_a < 0) {
    std::cout << "Courier A: Unreachable\n";
  } else {
    std::cout << "Courier A: " << couriers_time.mean_sec_a << '\n';
  }
  if (couriers_time.mean_sec_b < 0) {
    std::cout << "Courier B: Unreachable\n";
  } else {
    std::cout << "Courier B: " << couriers_time.mean_sec_b << '\n';
  }
}
