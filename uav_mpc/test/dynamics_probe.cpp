// Copyright (c) 2026 Ali-Eimaan.
// SPDX-License-Identifier: BSD-3-Clause
//
// Helper binary for test_acados_codegen.py (§10.3): reads (x, u) rows on stdin, writes the
// C++ QuadrotorDynamics f(x, u) row on stdout. Doubles as a manual debugging tool:
//
//   echo "0 0 1 ..." | ./dynamics_probe /path/to/params/x500_calibration.yaml

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "uav_mpc/quadrotor_dynamics.hpp"

#ifndef UAV_MPC_DEFAULT_AIRFRAME_PARAMS
#define UAV_MPC_DEFAULT_AIRFRAME_PARAMS ""
#endif

int main(int argc, char ** argv)
{
  std::string params_path;
  if (argc > 1) {
    params_path = argv[1];
  } else {
    params_path = UAV_MPC_DEFAULT_AIRFRAME_PARAMS;
  }
  if (params_path.empty()) {
    std::cerr << "usage: dynamics_probe <airframe_params.yaml>\n"
              << "reads lines of 'x0..x12 u0..u3' on stdin, prints 'f0..f12' per line\n";
    return 2;
  }

  const uav_mpc::QuadrotorParams params = uav_mpc::QuadrotorParams::fromYaml(params_path);
  uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion> dyn(params);

  constexpr int kNx = uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion>::kNx;
  constexpr int kNu = uav_mpc::QuadrotorDynamics<double, uav_mpc::AttitudeRep::Quaternion>::kNu;

  std::cout << std::setprecision(17);
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {continue;}
    std::istringstream iss(line);
    double values[kNx + kNu];
    bool ok = true;
    for (int i = 0; i < kNx + kNu; ++i) {
      if (!(iss >> values[i])) {
        ok = false;
        break;
      }
    }
    if (!ok) {continue;}
    Eigen::Matrix<double, kNx, 1> x;
    Eigen::Matrix<double, kNu, 1> u;
    for (int i = 0; i < kNx; ++i) {
      x(i) = values[i];
    }
    for (int i = 0; i < kNu; ++i) {
      u(i) = values[kNx + i];
    }
    const Eigen::Matrix<double, kNx, 1> xd = dyn.f(x, u);
    for (int i = 0; i < kNx; ++i) {
      if (i > 0) {std::cout << " ";}
      std::cout << xd(i);
    }
    std::cout << "\n";
  }
  return 0;
}
