#include <cmath>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include <vector>

#include "network.hpp"
#include "ortools/base/init_google.h"
#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"

#define NUM_CHARGERS 303
#define MAX_RANGE 320.0  // km
#define SPEED 105.0      // kmph
#define R 6356.752       // km
#define PI 3.141592653589793238462643383279502884197169399375105820
#define deg2rad(d) d* PI / 180.0

struct node {
  row city;
  double chargeTime;

  node(row c, double t) : city(c), chargeTime(t){};
};

struct stats {
  double tripTimeHrs;
  double computeTimeSecs;
};

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

double greatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
  // https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  double dLat = deg2rad((lat2 - lat1));
  double dLon = deg2rad((lon2 - lon1));

  // convert to radians
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);

  double a =
      pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  double c = 2 * asin(sqrt(a));
  return R * c;
}

row getCityFromString(std::string cityName) {
  for (auto city : network) {
    if (city.name == cityName) {
      return city;
    }
  }
}

double getTripTimeHrs(std::vector<node> path) {
  double tripTimeHrs = 0;
  for (int i = 1; i < path.size(); i++) {
    double drivingTime =
        greatCircleDistance(path[i - 1].city.lat, path[i - 1].city.lon,
                            path[i].city.lat, path[i].city.lon) /
        SPEED;
    double chargingTime = path[i - 1].chargeTime;
    tripTimeHrs += (drivingTime + chargingTime);
  }
  return tripTimeHrs;
}

bool verifyPath(std::vector<node> path) {
  bool validPath = true;
  double range = MAX_RANGE;
  for (int i = 1; i < path.size(); i++) {
    range -= greatCircleDistance(path[i - 1].city.lat, path[i - 1].city.lon,
                                 path[i].city.lat, path[i].city.lon);
    // std::cout << "range at " << path[i].city.name << " " << range << "\n";
    if (range < -1e-9) {
      std::cout << "ran out of range in city: " << path[i].city.name << " range: " << range << std::endl;
      validPath = false;
      break;
    }
    range += path[i].chargeTime * path[i].city.rate;
  }
  return validPath;
}

void prettyPrintSolution(std::vector<node> path, stats solutionStats) {
  // Pretty print the planned route and charging time at each city
  std::cout << "******************************** Planned path "
               "*********************************\n";
  printf("%-30s%-30s%-6s\n", "City", "Charging Rate (km/hr)",
         "Charging Time (hrs)");
  for (auto n : path) {
    printf("%-30s%-30f%-6f\n", n.city.name.c_str(), n.city.rate, n.chargeTime);
  }
  std::cout << "***************************************************************"
               "********"
               "********\n";
  std::cout << "Trip time: " << solutionStats.tripTimeHrs << " hrs"
            << "    "
            << "Path compute time: " << solutionStats.computeTimeSecs
            << " seconds" << std::endl;
  std::cout << "***************************************************************"
               "********"
               "********\n";
}

row findMinDeviationCity(row start, row goal, double range) {
  double sumDistances = std::numeric_limits<double>::infinity();
  row minDeviationCity;
  for (auto city : network) {
    if (city.name == start.name || city.name == goal.name) continue;
    double startToCity =
        greatCircleDistance(start.lat, start.lon, city.lat, city.lon);
    double cityToGoal =
        greatCircleDistance(city.lat, city.lon, goal.lat, goal.lon);
    if (startToCity + cityToGoal < sumDistances && startToCity < range) {
      sumDistances = startToCity + cityToGoal;
      minDeviationCity = city;
    }
  }
  return minDeviationCity;
}

std::vector<node> findBruteForcePath(std::string startCityName,
                                     std::string goalCityName) {
  std::vector<node> path;
  row start = getCityFromString(startCityName);
  row goal = getCityFromString(goalCityName);
  double range = MAX_RANGE;

  path.push_back(node(start, 0));
  while (greatCircleDistance(start.lat, start.lon, goal.lat, goal.lon) >
         range) {
    row minDeviationCity = findMinDeviationCity(start, goal, range);
    // Calculate range after driving to the minimum deviation city
    range -= greatCircleDistance(start.lat, start.lon, minDeviationCity.lat,
                                 minDeviationCity.lon);
    // Charge back to full range
    double chargeTimeMinDeviationCity =
        (MAX_RANGE - range) / minDeviationCity.rate;
    range = MAX_RANGE;

    path.push_back(node(minDeviationCity, chargeTimeMinDeviationCity));
    start = minDeviationCity;
  }

  path.push_back(node(goal, 0));
  return path;
}

std::vector<node> reevaluateChargingTimes(std::vector<node> path) {
  double range = MAX_RANGE;
  for (int i = 0; i < path.size() - 1; i++) {
    double dist =
        greatCircleDistance(path[i].city.lat, path[i].city.lon,
                            path[i + 1].city.lat, path[i + 1].city.lon);
    if (range >= dist) {
      range -= dist;
      path[i].chargeTime = 0.0;
      continue;
    }
    path[i].chargeTime = (dist - range) / path[i].city.rate;
    range = 0;
  }
  return path;
}

struct searchNode {
  row city;
  double deviation;

  searchNode(row c, double dev) : city(c), deviation(dev){};
};

class compareSearchNode {
 public:
  bool operator()(searchNode& node1, searchNode& node2) {
    return (node1.deviation > node2.deviation);
  }
};

std::vector<node> findMonteCarloPath(std::string startCityName,
                                     std::string goalCityName,
                                     int branchFactor) {
  row start = getCityFromString(startCityName);
  row goal = getCityFromString(goalCityName);
  double range = MAX_RANGE;

  std::vector<node> path;
  std::unordered_map<std::string, bool> visited;
  std::priority_queue<searchNode, std::vector<searchNode>, compareSearchNode>
      pq;

  path.push_back(node(start, 0.0));
  visited[start.name] = true;

  while (true) {
    for (auto n : network) {
      if (visited[n.name]) {
        continue;
      }
      if (n.name == start.name) {
        continue;
      }
      double startToCity =
          greatCircleDistance(start.lat, start.lon, n.lat, n.lon);
      if (startToCity > MAX_RANGE) {
        continue;
      }

      double cityToGoal = greatCircleDistance(n.lat, n.lon, goal.lat, goal.lon);
      double startToGoal =
          greatCircleDistance(start.lat, start.lon, goal.lat, goal.lon);
      double deviation = startToCity + cityToGoal - startToGoal;

      if (deviation > 2 * startToGoal) {
        continue;
      }

      pq.push(searchNode(n, deviation));
    }
    int iteration = 0;
    int rand_number = rand() % branchFactor;
    searchNode curr = pq.top();
    pq.pop();

    while (!pq.empty() && iteration < rand_number) {
      curr = pq.top();
      pq.pop();
      iteration += 1;
    }

    if (curr.city.name == goal.name) {
      path.push_back(node(goal, 0.0));
      break;
    }
    path.push_back(node(curr.city, 0.0));
    // visited[curr.city.name] = true;
    start = curr.city;
    pq = std::priority_queue<searchNode, std::vector<searchNode>,
                             compareSearchNode>();
  }

  return path;
}

std::vector<node> findOptimalChargingTimes(std::vector<node> path) {
  std::unique_ptr<operations_research::MPSolver> solver(
      operations_research::MPSolver::CreateSolver("GLOP"));

  std::vector<operations_research::MPVariable*> chargingTimes;
  const double infinity = solver->infinity();
  for (int i = 0; i < path.size() - 1; i++) {
    chargingTimes.push_back(solver->MakeNumVar(0.0, infinity, path[i].city.name));
  }
  LOG(INFO) << "Number of variables = " << solver->NumVariables();

  operations_research::MPObjective* const objective =
      solver->MutableObjective();
  for (size_t i = 0; i < path.size() - 1; i++) {
    objective->SetCoefficient(chargingTimes[i], 1);
  }
  objective->SetMinimization();

  double r0 = MAX_RANGE;
  double sum_d = 0;

  std::vector<operations_research::MPConstraint*> constraints;
  for (int i = 0; i < path.size() - 1; i++) {
    double di = greatCircleDistance(path[i].city.lat, path[i].city.lon, path[i+1].city.lat, path[i+1].city.lon);
    double lb = di - r0 + sum_d;
    double ub = MAX_RANGE - r0 + sum_d;
    sum_d += di;
    constraints.push_back(solver->MakeRowConstraint(lb, ub));
    for(int j = 0; j < path.size() - 1; j++) {
        constraints.back()->SetCoefficient(chargingTimes[j], (j <= i) ? path[j].city.rate : 0.0);
    }
  }

  const operations_research::MPSolver::ResultStatus result_status =
      solver->Solve();

  if (result_status != operations_research::MPSolver::OPTIMAL) {
    LOG(INFO) << "The problem does not have an optimal solution!";
    if (result_status == operations_research::MPSolver::FEASIBLE) {
      LOG(INFO) << "A potentially suboptimal solution was found";
    } else {
      LOG(INFO) << "The solver could not solve the problem.";
    }
  }

  LOG(INFO) << "Objective function value: "
            << std::to_string(objective->Value()) << std::endl;

  for (int i = 0; i < chargingTimes.size(); i++) {
    LOG(INFO) << path[i].city.name << " " << chargingTimes[i]->solution_value();
  }

  for(int i = 0; i < path.size() - 1; i++) {
    path[i].chargeTime = chargingTimes[i]->solution_value();
  }

  return path;
}

std::vector<node> runMonteCarlo(std::string startCityName,
                                std::string goalCityName, int branchFactor,
                                int maxIterations) {
  std::vector<node> bestPath;
  double bestTime = std::numeric_limits<double>::infinity();
  for (int i = 0; i < maxIterations; i++) {
    std::vector<node> path =
        findMonteCarloPath(startCityName, goalCityName, branchFactor);
    // path = reevaluateChargingTimes(path);
    path = findOptimalChargingTimes(path);
    double time = getTripTimeHrs(path);
    if (time < bestTime) {
      bestPath = path;
      bestTime = time;
    }
  }
  return bestPath;
}

int main(int argc, char** argv) {
  if (!input(argc, argv)) {
    std::cerr << "Error: requires correct initial and final supercharger names"
              << std::endl;
    return -1;
  }

  stats solutionStats;
  auto timeStart = std::chrono::high_resolution_clock::now();

  /*
  Approach 1:
  Given a start and goal, find a city which satisfies min(d(start, city) +
  d(city, goal)) Additionally impose the constraint that the d(start, city) <
  range At each city, charge fully to the maximum range of the vehicle Do
  this iteratively until d(city, goal) < range
  */
  // std::vector<node> path = findBruteForcePath(startCity, goalCity);

  /*
  Approach 2:
  Reset the charging times based on the found path. Set the charging time
  to just the amount of charge needed to get to the next city in the path
  */
  // path = reevaluateChargingTimes(path);

  /*
  Approach 3:
  - From the start city, run a monte carlo simulation by choosing the next
    city to go to in top 'branchFactor' number of mimimum deviation cities
  */
  std::vector<node> path = runMonteCarlo(startCity, goalCity, 3, 1000);

  /*
  Approach 4:
  The current charging strategy is to only charge enough to reach the next
  city in the path. This can be suboptimal if we can preemptively charge more
  in cities with faster charging rate

  Formulate it as an optimization problem:

  Variables:
  Let i = 1, 2, ... n be the index noting the cities in the path
  Let t[i] denote the time spent in charging at city i
  Let k[i] denote the rate of charging at city i
  Let d[i][j] be the distance between city i and city j
  Let r[i] be the range at city i when departing city i after charging
  Let R be the maximum range of the vehicle

  Objective:
  minimize t[1] + t[2] + ... + t[i] + ... + t[n]

  Constraints:
  t[i] >= 0
  0 <= r[i] <= R

  r[1] = R
  r[2] = r[1] - d[1][2] + k[1] * t[1]
  .
  .
  r[i] = r[i-1] - d[i-1][i] + k[i] * t[i]

  r[i] >= d[i][i+1]

  Solution:
  This is a linear programming problem which follows the structure

  min kx s.t Ax >= b and x >= 0

  */
  path = findOptimalChargingTimes(path);

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
