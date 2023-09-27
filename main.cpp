#include <cmath>
#include <iomanip>
#include <vector>
#include <queue>

#include "network.h"

#define NUM_CHARGERS 303
#define INIT_RANGE 320.0  // km
#define SPEED 105.0       // kmph
#define R 6356.752        // km
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
  double range = INIT_RANGE;
  for (int i = 1; i < path.size(); i++) {
    range -= greatCircleDistance(path[i - 1].city.lat, path[i - 1].city.lon,
                                 path[i].city.lat, path[i].city.lon);
    // std::cout << "range at " << path[i].city.name << " " << range << "\n";
    if (range < 0) {
      validPath = false;
      break;
    }
    range += path[i].chargeTime * path[i].city.rate;
  }
  return validPath;
}

void prettyPrintSolution(std::vector<node> path, stats solutionStats) {
  // Pretty print the planned route and charging time at each city
  std::cout << "**************************** Planned path "
               "*****************************\n";
  for (auto n : path) {
    std::cout << "City: " << n.city.name << std::setw(50 - n.city.name.size())
              << " Charging Time: " << n.chargeTime << " hr" << std::endl;
  }
  std::cout << "***************************************************************"
               "********\n";
  std::cout << "Trip time: " << solutionStats.tripTimeHrs << " hrs"
            << "    "
            << "Path compute time: " << solutionStats.computeTimeSecs
            << " seconds" << std::endl;
  std::cout << "***************************************************************"
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
  double range = INIT_RANGE;

  path.push_back(node(start, 0));
  while (greatCircleDistance(start.lat, start.lon, goal.lat, goal.lon) >
         range) {
    row minDeviationCity = findMinDeviationCity(start, goal, range);
    // Calculate range after driving to the minimum deviation city
    range -= greatCircleDistance(start.lat, start.lon, minDeviationCity.lat,
                                 minDeviationCity.lon);
    // Charge back to full range
    double chargeTimeMinDeviationCity =
        (INIT_RANGE - range) / minDeviationCity.rate;
    range = INIT_RANGE;

    path.push_back(node(minDeviationCity, chargeTimeMinDeviationCity));
    start = minDeviationCity;
  }

  path.push_back(node(goal, 0));
  return path;
}

std::vector<node> reevaluateChargingTimes(std::vector<node> path) {
  double range = INIT_RANGE;
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
  double distToGoal;
  double chargeTime;

  searchNode(row c, double dev, double dist, double ct) : city(c), deviation(dev), distToGoal(dist), chargeTime(ct){};
};

class compareSearchNode
{
public:
    bool operator()(searchNode &node1, searchNode &node2) {
      return (node1.deviation > node2.deviation);
    }

};

std::vector<node> findMonteCarloPath(std::string startCityName, std::string goalCityName, int branchFactor) {
  std::vector<node> path;
  row start = getCityFromString(startCityName);
  row goal = getCityFromString(goalCityName);
  double range = INIT_RANGE;

  std::vector<bool> visited(network.size(), false);
  std::priority_queue<searchNode, std::vector<searchNode>, compareSearchNode> pq;

  pq.push(searchNode(start, 0.0, greatCircleDistance(start.lat, start.lon, goal.lat, goal.lon), 0.0));

  bool done = false;
  // while (!done) {

  // }
}

int main(int argc, char** argv) {
  if (!input(argc, argv)) {
    std::cerr << "Error: requires correct initial and final supercharger names"
              << std::endl;
    return -1;
  }

  stats solutionStats;
  auto timeStart = std::chrono::high_resolution_clock::now();

  // Approach 1:
  // Given a start and goal, find a city which satisfies min(d(start, city) +
  // d(city, goal)) Additionally impose the constraint that the d(start, city) <
  // range At each city, charge fully to the maximum range of the vehicle Do
  // this iteratively until d(city, goal) < range
  std::vector<node> path = findBruteForcePath(startCity, goalCity);

  // Approach 2:
  // Reset the charging times based on the found path. Set the charging time
  // to just the amount of charge needed to get to the next city in the path
  std::vector<node> reevaluatedPath = reevaluateChargingTimes(path);
  path = reevaluatedPath;

  // Approach 3:
  // - From both start and the goal, add the cities within range to their
  // respective priority queues.
  // - Set the key of the priority queue to be (d(start, city) + d(city, goal) -
  // d(start, goal))
  // - Expand from both the start and goal priority queues until there is an
  // intersection between the queues.
  // - Do a Monte Carlo from start to goal and find the minimum time path

  auto timeEnd = std::chrono::high_resolution_clock::now();
  auto timeTaken = std::chrono::duration_cast<std::chrono::duration<double>>(
      timeEnd - timeStart);
  solutionStats.computeTimeSecs = timeTaken.count();
  solutionStats.tripTimeHrs = getTripTimeHrs(path);

  std::cout << "Path valid? : " << verifyPath(path) << std::endl;
  prettyPrintSolution(path, solutionStats);

  return 0;
}
