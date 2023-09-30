#include "path.hpp"

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
  /*
  Given a start and goal, find a city which satisfies min(d(start, city) +
  d(city, goal)) Additionally impose the constraint that the d(start, city) <
  range At each city, charge fully to the maximum range of the vehicle Do
  this iteratively until d(city, goal) < range
  */
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

std::vector<node> findMonteCarloPath(std::string startCityName,
                                     std::string goalCityName,
                                     int branchFactor) {
  /*
  From the start city, run a monte carlo simulation by choosing the next
  city to go to in top 'branchFactor' number of mimimum deviation cities
  */
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

std::vector<node> runMonteCarlo(std::string startCityName,
                                std::string goalCityName, int branchFactor,
                                int maxIterations, bool optimal) {
  std::vector<node> bestPath;
  double bestTime = std::numeric_limits<double>::infinity();
  for (int i = 0; i < maxIterations; i++) {
    std::vector<node> path =
        findMonteCarloPath(startCityName, goalCityName, branchFactor);

    if (optimal) {
      path = findOptimalChargingTimes(path, false);
    } else {
      path = findGreedyChargingTimes(path);
    }

    double time = getTripTimeHrs(path);
    if (time < bestTime) {
      bestPath = path;
      bestTime = time;
    }
  }
  return bestPath;
}
