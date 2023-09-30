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

enum Method {
  /*
  Find the minimum deviation city which is reachable with a fully charged battery which satisfies
  minimum(d(start, city) + d(city, goal) - d(start, goal)) and d(start, city) < MAX_RANGE
  At each city, charge the battery to full capacity.
  */
  BRUTE_FORCE_CHARGE_FULL,
  /*
  Find the minimum deviation city which is reachable with a fully charged battery which satisfies
  minimum(d(start, city) + d(city, goal) - d(start, goal)) and d(start, city) < MAX_RANGE
  At each city, charge the battery just enough to reach the next city.
  */
  BRUTE_FORCE_CHARGE_GREEDY,
  /*
  Find the minimum deviation city which is reachable with a fully charged battery which satisfies
  minimum(d(start, city) + d(city, goal) - d(start, goal)) and d(start, city) < MAX_RANGE
  Find the optimal charging strategy which minimizes the sum of time spent charging at each city
  */
  BRUTE_FORCE_CHARGE_OPTIMAL,
  /*
  Run a Monte Carlo simulation to randomly find different paths from start to goal and evaluate
  the total time taken to complete the trip. Keep track of the path which takes the minimum time.
  At each city, charge the battery just enough to reach the next city.
  */
  MONTE_CARLO_CHARGE_GREEDY,
  /*
  Run a Monte Carlo simulation to randomly find different paths from start to goal and evaluate
  the total time taken to complete the trip. Keep track of the path which takes the minimum time.
  Find the optimal charging strategy which minimizes the sum of time spent charging at each city
  */
  MONTE_CARLO_CHARGE_OPTIMAL,
};

int main(int argc, char** argv) {
  if (!input(argc, argv)) {
    std::cerr << "Error: requires correct initial and final supercharger names"
              << std::endl;
    return -1;
  }

  Method method = MONTE_CARLO_CHARGE_OPTIMAL;

  stats solutionStats;
  auto timeStart = std::chrono::high_resolution_clock::now();

  std::vector<node> path;

  switch (method) {
    case BRUTE_FORCE_CHARGE_FULL:
      path = findBruteForcePath(startCity, goalCity);
      break;
    case BRUTE_FORCE_CHARGE_GREEDY:
      path = findBruteForcePath(startCity, goalCity);
      path = findGreedyChargingTimes(path);
      break;
    case BRUTE_FORCE_CHARGE_OPTIMAL:
      path = findBruteForcePath(startCity, goalCity);
      path = findOptimalChargingTimes(path, false);
      break;
    case MONTE_CARLO_CHARGE_GREEDY:
      path = runMonteCarlo(startCity, goalCity, 3, 1000, false);
      break;
    case MONTE_CARLO_CHARGE_OPTIMAL:
      path = runMonteCarlo(startCity, goalCity, 3, 1000, true);
      break;
    default:
      std::cout << "Invalid method!\n";
      break;
  }

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
