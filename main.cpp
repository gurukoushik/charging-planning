#include <cmath>
#include <iomanip>
#include <vector>

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

std::vector<node> findPath(std::string startCityName,
                           std::string goalCityName) {
  std::vector<node> path;
  row start = getCityFromString(startCity);
  row goal = getCityFromString(goalCity);
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

void prettyPrintSolution(std::vector<node> path, stats solutionStats) {
  // Pretty print the planned route and charging time at each city
  std::cout << "**************************** Planned route "
               "****************************\n";
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

int main(int argc, char** argv) {
  if (!input(argc, argv)) {
    std::cerr << "Error: requires correct initial and final supercharger names"
              << std::endl;
    return -1;
  }
  // Approach 1:
  // Given a start and goal, find a city which satisfies min(d(start, city) +
  // d(city, goal)) Additionally impose the constraint that the d(start, city) <
  // range At each city, charge fully to the maximum range of the vehicle Do
  // this iteratively until d(city, goal) < range
  stats solutionStats;
  auto timeStart = std::chrono::high_resolution_clock::now();

  std::vector<node> path = findPath(startCity, goalCity);

  auto timeEnd = std::chrono::high_resolution_clock::now();
  auto timeTaken = std::chrono::duration_cast<std::chrono::duration<double>>(
      timeEnd - timeStart);
  solutionStats.computeTimeSecs = timeTaken.count();
  solutionStats.tripTimeHrs = getTripTimeHrs(path);

  prettyPrintSolution(path, solutionStats);

  return 0;
}
