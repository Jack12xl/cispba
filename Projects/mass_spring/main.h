#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <random>
#include <chrono>
#include <unordered_set>
#include <iomanip>
#include <set>

#include "SimulationDriver.h"
#include "utils.h"

using T = float;
const int dim = 3;
using TV = Eigen::Matrix<T, dim, 1>;

int numPoint = 0;
T uniform_M = 1.0f;