#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <Eigen/Dense>

namespace rsu {

struct RSUParams {
  // Geometry constants (units: mm or m — just be consistent)
  std::array<Eigen::Vector3d, 2> a_W;  // a_i in W frame
  std::array<Eigen::Vector3d, 2> b_F;  // b_i in F frame
  std::array<double, 2> c;             // crank length
  std::array<double, 2> r;             // rod length
  std::array<double, 2> psi;           // motor mount angle about z (rad) - optional, can be 0

  // Numerical settings
  double eps = 1e-9;
};

struct SolveResult {
  bool feasible = false;

  // Selected motor angles (rad)
  std::array<double, 2> alpha = {0.0, 0.0};

  // Which branch selected per leg: 0 or 1
  std::array<int, 2> branch = {0, 0};

  // Debug / diagnostics
  std::array<double, 2> k = {0.0, 0.0};
  std::array<double, 2> rho = {0.0, 0.0};
  std::array<double, 2> arg = {0.0, 0.0};        // asin argument (clamped)
  std::array<double, 2> residual = {0.0, 0.0};   // equation residual per leg

  // internal vectors (optional to inspect)
  std::array<Eigen::Vector3d, 2> d = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> d_hat = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
};

class RSUSolver {
public:
  explicit RSUSolver(RSUParams params);

  // roll=phi, pitch=theta (rad). Yaw is assumed 0.
  // prev_alpha: for continuity-based branch selection (optional)
  SolveResult solve(double roll, double pitch,
                    const std::optional<std::array<double,2>>& prev_alpha = std::nullopt) const;

  const RSUParams& params() const { return params_; }

private:
  RSUParams params_;

  static Eigen::Matrix3d Rx(double a);
  static Eigen::Matrix3d Ry(double a);
  static Eigen::Matrix3d Rz(double a);

  static double wrapToPi(double x);
  static double clamp(double v, double lo, double hi);

  // For your hardware: motor rotates about +Y axis in W (right-hand rule).
  // Closed-form equation will use (x,z) components in motor-aligned frame.
  static void computeAlphaCandidates_Yaxis(
      double dx, double dz, double k,
      double eps,
      std::array<double,2>& alpha_cand,
      std::array<bool,2>& cand_valid,
      double& rho_out,
      double& arg_out
  );
};

} // namespace rsu

