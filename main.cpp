#include "network.hpp"
#include "path.hpp"
#include "utils.hpp"

std::string startCity, goalCity;

bool input(int argc, char** argv) {
  if (argc != 3) return false;
  startCity = argv[1];
  goalCity = argv[2];

  int startIndex = 0, goalIndex = 0;
  while (startIndex < network.size() && network[startIndex].name != startCity)
    ++startIndex;
  while (goalIndex < network.size() && network[goalIndex].name != goalCity)
    ++goalIndex;
  if (startIndex >= network.size() || goalIndex >= network.size()) return false;

  return true;
}

int main(int argc, char** argv) {
  if (!input(argc, argv)) {
    std::cerr << "Error: requires correct initial and final supercharger names"
              << std::endl;
    return -1;
  }

  stats solutionStats;
  auto timeStart = std::chrono::high_resolution_clock::now();

  // std::vector<node> path = findBruteForcePath(startCity, goalCity);
  // path = reevaluateChargingTimes(path);
  std::vector<node> path = runMonteCarlo(startCity, goalCity, 3, 1000);

  auto timeEnd = std::chrono::high_resolution_clock::now();
  auto timeTaken = std::chrono::duration_cast<std::chrono::duration<double>>(
      timeEnd - timeStart);
  solutionStats.computeTimeSecs = timeTaken.count();
  solutionStats.tripTimeHrs = getTripTimeHrs(path);

  std::cout << "\nPath valid : " << (verifyPath(path) ? "yes" : "no")
            << std::endl;
  prettyPrintSolution(path, solutionStats);

  return 0;
}
