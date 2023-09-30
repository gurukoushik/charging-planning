# Route Planning with Charging Stops

Given a network of cities with their (lattitude, longitude) coordinates
and the rate of charging (km/hr) at each city, find a route between a
given start and goal city which takes the least amount of time. The 
assumptions made are that you start with a vehicle that is fully charged
and moves at a constant speed along the great circle distance along the
surface of the Earth. At any point during the trip, the range of the
vehicle cannot drop below zero.

## Dependencies

[Google OR-Tools](https://developers.google.com/optimization/install/cpp)

## Usage

Clone the repository

```bash
git clone git@github.com:gurukoushik/charging-planning.git
cd charging-planning
```

Build and run the solution

```bash
# Build the targets
make build

# Short route
build/plan Council_Bluffs_IA Cadillac_MI

# Long route
build/plan San_Diego_CA Brooklyn_NY
```

## Source

Data and boilerplate code taken from [repo](https://github.com/Franceshe/Tesla-Autopilot-Challenge).
