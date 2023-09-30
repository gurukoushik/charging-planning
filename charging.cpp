#include "charging.hpp"

std::vector<node> findGreedyChargingTimes(std::vector<node> path) {
  /*
  Reset the charging times based on the found path. Set the charging time
  to just the amount of charge needed to get to the next city in the path
  */
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

std::vector<node> findOptimalChargingTimes(std::vector<node> path,
                                           bool verbose) {
  /*
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
  std::unique_ptr<operations_research::MPSolver> solver(
      operations_research::MPSolver::CreateSolver("GLOP"));

  std::vector<operations_research::MPVariable*> chargingTimes;
  const double infinity = solver->infinity();
  for (int i = 0; i < path.size() - 1; i++) {
    chargingTimes.push_back(
        solver->MakeNumVar(0.0, infinity, path[i].city.name));
  }

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
    double di = greatCircleDistance(path[i].city.lat, path[i].city.lon,
                                    path[i + 1].city.lat, path[i + 1].city.lon);
    double lb = di - r0 + sum_d;
    double ub = MAX_RANGE - r0 + sum_d;
    sum_d += di;
    constraints.push_back(solver->MakeRowConstraint(lb, ub));
    for (int j = 0; j < path.size() - 1; j++) {
      constraints.back()->SetCoefficient(chargingTimes[j],
                                         (j <= i) ? path[j].city.rate : 0.0);
    }
  }

  const operations_research::MPSolver::ResultStatus result_status =
      solver->Solve();

  if (verbose) {
    LOG(INFO) << "Number of variables = " << solver->NumVariables();
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
      LOG(INFO) << path[i].city.name << " "
                << chargingTimes[i]->solution_value();
    }
  }

  for (int i = 0; i < path.size() - 1; i++) {
    path[i].chargeTime = chargingTimes[i]->solution_value();
  }

  return path;
}
