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
#include <set>

#include "SimulationDriver.h"
#include "utils.h"

// resolution
const int X_RES = 32;
const int Y_RES = 32;
int numPoint = X_RES * Y_RES;

using T = float;
constexpr int dim = 3;
using TV = Eigen::Matrix<T, dim, 1>;


// set up mass spring system
T youngs_modulus = 256.0f;
T damping_coeff = 16.0f;
T dt = 0.001;

const int total_sequence_num = 120;

T uniform_M = 0.1f;

float X_offset = 0.0f, Y_offset = 0.0f, Z_offset = 0.0f;
float X_d = 0.1f, Y_d = 0.1f;

// control the cloth to move
#define Z_move 1
#define X_move 1
const float Z_amplitude = 1.0f;
const float Z_omega = 0.6f * PI;

const float X_amplitude = 1.5f;
const float X_omega = 0.6f * PI;

