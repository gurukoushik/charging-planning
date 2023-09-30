#pragma once
#include <vector>

#include "ortools/base/init_google.h"
#include "ortools/base/logging.h"
#include "ortools/linear_solver/linear_solver.h"
#include "utils.hpp"

std::vector<node> reevaluateChargingTimes(std::vector<node> path);
std::vector<node> findOptimalChargingTimes(std::vector<node> path,
                                           bool verbose);
