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

#include "SimulationDriver.h"

// resolution
const int X_RES = 64;
const int Y_RES = 64;
const int numPoint = X_RES * Y_RES;

using T = float;
constexpr int dim = 3;
using TV = Eigen::Matrix<T, dim, 1>;


// set up mass spring system
T youngs_modulus = 0;
T damping_coeff = 0;
T dt = 0;

T uniform_M = 1.0f;