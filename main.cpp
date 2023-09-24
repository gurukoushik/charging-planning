#include "network.h"
#include <cmath>

#define NUM_CHARGERS 303
#define INIT_CHARGE 320.0
#define SPEED 105.0
#define R 6356.752
#define PI 3.141592653589793238462643383279502884197169399375105820
#define deg2rad(d) d*PI/180.0

std::string start, goal;

bool input(int argc, char** argv) {
    if (argc != 3) return false;
    start = argv[1];
    goal = argv[2];
    
    int startIndex = 0, goalIndex = 0;
    while (startIndex < network.size() && network[startIndex].name != start) ++startIndex;
    while (goalIndex < network.size() && network[goalIndex].name != goal) ++goalIndex;
    if (startIndex >= network.size() || goalIndex >= network.size()) return false;

    return true;
}

double greatCircleDistance(double lat1, double lon1, double lat2, double lon2, double radius) {
    // FIXME
    return 0.0;    
}

int main(int argc, char** argv)
{
    if (!input(argc, argv)) {
        std::cerr << "Error: requires correct initial and final supercharger names" << std::endl;
        return -1;
    }
    
    return 0;
}
