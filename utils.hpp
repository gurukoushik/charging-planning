#pragma once
#include <cmath>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include <vector>

#include "network.hpp"

#define NUM_CHARGERS 303
#define MAX_RANGE 320.0  // km
#define SPEED 105.0      // kmph
#define RADIUS 6356.752  // km
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

double greatCircleDistance(double lat1, double lon1, double lat2, double lon2);
row getCityFromString(std::string cityName);
double getTripTimeHrs(std::vector<node> path);
bool verifyPath(std::vector<node> path);
void prettyPrintSolution(std::vector<node> path, stats solutionStats);
