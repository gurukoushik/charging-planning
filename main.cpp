#include "network.h"
#include <cmath>
#include <vector>

#define NUM_CHARGERS 303
#define INIT_CHARGE 320.0
#define SPEED 105.0
#define R 6356.752
#define PI 3.141592653589793238462643383279502884197169399375105820
#define deg2rad(d) d*PI/180.0

struct node {
    row city;
    double chargeTime;

    node(row c, double t): city(c), chargeTime(t) {};
};

std::string startCity, goalCity;

bool input(int argc, char** argv) {
    if (argc != 3) return false;
    startCity = argv[1];
    goalCity = argv[2];
    
    int startIndex = 0, goalIndex = 0;
    while (startIndex < network.size() && network[startIndex].name != startCity) ++startIndex;
    while (goalIndex < network.size() && network[goalIndex].name != goalCity) ++goalIndex;
    if (startIndex >= network.size() || goalIndex >= network.size()) return false;

    return true;
}

double greatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
    // https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);

    // convert to radians
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double c = 2 * asin(sqrt(a));
    return R * c;
}

row getCityFromString(std::string cityName) {
    for (auto city: network) {
        if (city.name == cityName){
            return city;
        }
    }
}

std::vector<node> findPath(std::string startCityName, std::string goalCityName) {
    std::vector<node> path;
    row start = getCityFromString(startCity);
    row goal = getCityFromString(goalCity);
    double range = INIT_CHARGE;

    path.push_back(node(start, 0));

    while (greatCircleDistance(start.lat, start.lon, goal.lat, goal.lon) > range) {
        // for (auto city: network) {

        // }
        break;
    }

    path.push_back(node(goal, 0));
    return path;
}

int main(int argc, char** argv)
{
    if (!input(argc, argv)) {
        std::cerr << "Error: requires correct initial and final supercharger names" << std::endl;
        return -1;
    }
    // Approach 1: 
    // Given a start and goal, find a city which satisfies min(d(start, city) + d(city, goal))
    // Additionally impose the constraint that the d(start, city) < range
    // Do this iteratively until d(city, goal) < range
    std::vector<node> path = findPath(startCity, goalCity);

    for(auto n: path){
        std::cout << "City: " << n.city.name << " Charging Time: " << n.chargeTime << std::endl; 
    }

    return 0;
}
