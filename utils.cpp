#include "utils.hpp"

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
  return RADIUS * c;
}

row getCityFromString(std::string cityName) {
  row city;
  for (auto n : network) {
    if (n.name == cityName) {
      city = n;
    }
  }
  return city;
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
      std::cout << "ran out of range in city: " << path[i].city.name
                << " range: " << range << std::endl;
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
