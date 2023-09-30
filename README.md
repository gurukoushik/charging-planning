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

### Clone the repository

```bash
git clone git@github.com:gurukoushik/charging-planning.git
cd charging-planning
```

### Build and run the solution

```bash
# Build the targets
make build

# Short route
build/plan Council_Bluffs_IA Cadillac_MI

# Long route
build/plan San_Diego_CA Brooklyn_NY
```

### Sample output

```bash
❯ build/plan San_Diego_CA Brooklyn_NY

Path valid : yes
******************************** Planned path *********************************
City                          Charging Rate (km/hr)         Charging Time (hrs)
San_Diego_CA                  102.000000                    0.000000
Quartzsite_AZ                 123.000000                    0.892724
Wickenburg_AZ                 164.000000                    1.951220
Holbrook_AZ                   132.000000                    0.567360
Gallup_NM                     161.000000                    1.275410
Albuquerque_NM                175.000000                    1.828571
Tucumcari_NM                  147.000000                    1.750891
Shamrock_TX                   173.000000                    1.849711
Oklahoma_City_OK              87.000000                     1.797094
Wichita_KS                    154.000000                    2.077922
Topeka_KS                     122.000000                    1.721424
Columbia_MO                   109.000000                    1.214517
St._Charles_MO                115.000000                    2.479538
Terre_Haute_IN                146.000000                    2.191781
Indianapolis_IN               91.000000                     0.174901
East_Liberty_OH               145.000000                    2.023761
Cranberry_PA                  148.000000                    2.162162
Harrisburg_PA                 141.000000                    1.438109
Brooklyn_NY                   115.000000                    0.000000
*******************************************************************************
Trip time: 67.2394 hrs    Path compute time: 0.964883 seconds
*******************************************************************************
```

## Source

Data and boilerplate code taken from [repo](https://github.com/Franceshe/Tesla-Autopilot-Challenge).
