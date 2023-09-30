#pragma once
#include "charging.hpp"
#include "utils.hpp"

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

row findMinDeviationCity(row start, row goal, double range);
std::vector<node> findBruteForcePath(std::string startCityName,
                                     std::string goalCityName);
std::vector<node> findMonteCarloPath(std::string startCityName,
                                     std::string goalCityName,
                                     int branchFactor);
std::vector<node> runMonteCarlo(std::string startCityName,
                                std::string goalCityName, int branchFactor,
                                int maxIterations);