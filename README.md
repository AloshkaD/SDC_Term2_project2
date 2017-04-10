# Unscented Kalman Filter Project  
Accuracy using file 1:
Accuracy - RMSE:
0.0573841
0.0397915
0.484232
0.459976

---
<iframe width="900" height="800" frameborder="0" scrolling="no" src="https://plot.ly/~AloshkaD/1.embed"></iframe>

Accuracy using file 2:
Accuracy - RMSE:
0.190818
0.1922
0.51473
0.436658

<iframe width="900" height="800" frameborder="0" scrolling="no" src="https://plot.ly/~AloshkaD/3.embed"></iframe>

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

I've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

I'm following [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

Sample lidar and Radar data is provided 

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

