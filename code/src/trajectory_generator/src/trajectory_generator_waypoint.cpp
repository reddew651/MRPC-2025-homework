#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(m, 3 * p_num1d);

  int dim = m * p_num1d;
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(dim, dim);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(dim, 3);

  auto factorial = [](int n) {
    double res = 1.0;
    for (int i = 1; i <= n; ++i) res *= i;
    return res;
  };

  // Initial conditions (p, v, a, j, ...) at t=0 of first segment
  for (int i = 0; i < d_order; ++i) {
    M(i, i) = factorial(i);
  }
  b.row(0) = Path.row(0);
  b.row(1) = Vel.row(0);
  b.row(2) = Acc.row(0);
  // Higher order derivatives at start are 0

  // Continuity constraints and intermediate waypoints
  for (int i = 0; i < m - 1; ++i) {
    double t = Time(i);
    int idx = (i + 1) * p_num1d;

    // Position constraint at end of segment i
    for (int j = 0; j < p_num1d; ++j) {
      M(idx - d_order, i * p_num1d + j) = pow(t, j);
    }
    b.row(idx - d_order) = Path.row(i + 1);

    // Continuity constraints at junction i / i+1
    for (int k = 0; k < p_order; ++k) {
      // Derivative k of segment i at t=T_i
      for (int j = k; j < p_num1d; ++j) {
        double val = factorial(j) / factorial(j - k) * pow(t, j - k);
        M(idx - d_order + 1 + k, i * p_num1d + j) = val;
      }
      // Derivative k of segment i+1 at t=0
      M(idx - d_order + 1 + k, (i + 1) * p_num1d + k) = -factorial(k);
    }
  }

  // Terminal conditions at t=T_m of last segment
  double t_last = Time(m - 1);
  int last_idx = (m - 1) * p_num1d;
  for (int k = 0; k < d_order; ++k) {
    for (int j = k; j < p_num1d; ++j) {
      M(dim - d_order + k, last_idx + j) = factorial(j) / factorial(j - k) * pow(t_last, j - k);
    }
  }
  b.row(dim - d_order) = Path.row(m);
  b.row(dim - d_order + 1) = Vel.row(1);
  b.row(dim - d_order + 2) = Acc.row(1);

  // Solve
  Eigen::MatrixXd coefficientMatrix = M.lu().solve(b);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < 3; j++) {
      PolyCoeff.block(i, j * p_num1d, 1, p_num1d) = coefficientMatrix.block(i * p_num1d, j, p_num1d, 1).transpose();
    }
  }

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}