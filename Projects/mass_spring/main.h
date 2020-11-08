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
T total_M = T(1);
//T uniform_M = 1.0f;

std::vector<T> m_bunny_young_modules = {0.1f, 1.0f, 10.0f, 100.0f, 1000.0f };